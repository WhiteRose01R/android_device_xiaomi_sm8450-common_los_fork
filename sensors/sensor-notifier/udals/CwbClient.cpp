/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsCwbClient"

#include "CwbClient.h"

#include <android-base/logging.h>
#include <android/hardware_buffer.h>
#include <display_config.h>
#include <pthread.h>
#include <vndk/hardware_buffer.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

// Qualcomm HWC primary display identifier; HWC rejects display 0 for CWB
constexpr uint32_t kPrimaryDisplayId = 1;

std::array<float, 256> initGammaLut() {
    std::array<float, 256> lut{};
    for (size_t i = 0; i < 256; ++i) {
        lut[i] = std::pow(static_cast<float>(i) / 255.0f, 2.2f);
    }
    return lut;
}

const std::array<float, 256> kGammaLut = initGammaLut();

struct ScopedHardwareBufferLock {
    ScopedHardwareBufferLock(AHardwareBuffer* buffer, uint64_t usage)
        : mBuffer(buffer), mLocked(false), mAddr(nullptr) {
        if (mBuffer != nullptr) {
            int err = AHardwareBuffer_lock(mBuffer, usage, -1, nullptr, &mAddr);
            mLocked = (err == 0 && mAddr != nullptr);
        }
    }

    ~ScopedHardwareBufferLock() {
        if (mLocked && mBuffer != nullptr) {
            AHardwareBuffer_unlock(mBuffer, nullptr);
        }
    }

    ScopedHardwareBufferLock(const ScopedHardwareBufferLock&) = delete;
    ScopedHardwareBufferLock& operator=(const ScopedHardwareBufferLock&) = delete;

    bool isLocked() const { return mLocked; }
    void* address() const { return mAddr; }

  private:
    AHardwareBuffer* mBuffer;
    bool mLocked;
    void* mAddr;
};

}  // namespace

class CwbCallbackHandler : public DisplayConfig::ConfigCallback {
  public:
    void NotifyCWBBufferDone(int32_t error, const native_handle_t* /* buffer */) override {
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mPending || mActiveRequestId == 0) {
            return;
        }
        mError = error;
        mDone = true;
        mPending = false;
        mActiveRequestId = 0;
        mCv.notify_all();
    }

    void prepareForCapture(uint64_t requestId) {
        std::lock_guard<std::mutex> lock(mMutex);
        mDone = false;
        mError = 0;
        mPending = true;
        mActiveRequestId = requestId;
    }

    void cancelPending() {
        std::lock_guard<std::mutex> lock(mMutex);
        mPending = false;
        mActiveRequestId = 0;
    }

    bool isPending() const {
        std::lock_guard<std::mutex> lock(mMutex);
        return mPending;
    }

    bool consumeIfDone(int32_t* outError) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mDone) {
            return false;
        }
        *outError = mError;
        mDone = false;
        return true;
    }

    bool waitForDone(std::chrono::milliseconds timeout, int32_t* outError) {
        std::unique_lock<std::mutex> lock(mMutex);
        if (!mCv.wait_for(lock, timeout, [this] { return mDone || mAborted; })) {
            return false;
        }
        if (mAborted) {
            return false;
        }
        *outError = mError;
        mDone = false;
        return true;
    }

    void abort() {
        std::lock_guard<std::mutex> lock(mMutex);
        mAborted = true;
        mCv.notify_all();
    }

    bool waitForDrain(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mMutex);
        return mCv.wait_for(lock, timeout, [this] { return !mPending; });
    }

    void forceReset() {
        std::lock_guard<std::mutex> lock(mMutex);
        mDone = false;
        mPending = false;
        mError = 0;
        mAborted = false;
        mActiveRequestId = 0;
    }

  private:
    mutable std::mutex mMutex;
    std::condition_variable mCv;
    bool mDone = false;
    bool mPending = false;
    bool mAborted = false;
    uint64_t mActiveRequestId = 0;
    int32_t mError = 0;
};

CwbClient::CwbClient(UdalsRoi roi, std::shared_ptr<UDalsBleedCompensator> compensator)
    : mRoi(roi),
      mRoiWidth(roi.right - roi.left),
      mRoiHeight(roi.bottom - roi.top),
      mCenterRelX(mRoiWidth / 2),
      mCenterRelY(mRoiHeight / 2),
      mCompensator(std::move(compensator)),
      mRunning(false),
      mPaused(true),
      mCallbackHandler(std::make_shared<CwbCallbackHandler>()) {
    int32_t maxRadius = std::min(mRoiWidth, mRoiHeight) / 2;
    mRadius1 = static_cast<int32_t>(std::lround(maxRadius * 0.38f));
    mRadius2 = static_cast<int32_t>(std::lround(maxRadius * 0.72f));
    mRadius3 = maxRadius;
    mRadius1Sq = mRadius1 * mRadius1;
    mRadius2Sq = mRadius2 * mRadius2;
    mRadius3Sq = mRadius3 * mRadius3;
}

CwbClient::~CwbClient() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mRunning = false;
        mPaused = false;
    }

    if (mCallbackHandler != nullptr) {
        mCallbackHandler->abort();
    }

    mCv.notify_all();

    if (mWorker.joinable()) {
        mWorker.join();
    }

    releaseCwb();
}

bool CwbClient::ensureCwbInitialized() {
    if (mCwbClient != nullptr && mCwbBufferHandles[0] != nullptr &&
        mCwbBufferHandles[1] != nullptr) {
        return true;
    }

    auto now = std::chrono::steady_clock::now();
    auto minInterval = (mInitFailures >= 15) ? std::chrono::seconds(5) : std::chrono::seconds(1);
    if (mInitFailures > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(
                                     now - mLastInitAttempt) < minInterval) {
        return false;
    }
    mLastInitAttempt = now;

    if (mCwbClient == nullptr) {
        DisplayConfig::ClientInterface* client = nullptr;
        if (DisplayConfig::ClientInterface::Create("sensor-notifier", mCallbackHandler.get(),
                                                   &client) != 0 ||
            client == nullptr) {
            mInitFailures++;
            LOG(WARNING) << "DisplayConfig ClientInterface Create failed (attempt " << mInitFailures
                         << ")";
            releaseCwb();
            return false;
        }
        mCwbClient = client;
    }

    AHardwareBuffer_Desc desc = {
            .width = static_cast<uint32_t>(mRoiWidth),
            .height = static_cast<uint32_t>(mRoiHeight),
            .layers = 1,
            .format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM,
            .usage = AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN | AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT |
                     AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY,
            .stride = 0,
            .rfu0 = 0,
            .rfu1 = 0,
    };

    for (size_t i = 0; i < kBufferCount; ++i) {
        if (mCwbBuffers[i] == nullptr) {
            if (AHardwareBuffer_allocate(&desc, &mCwbBuffers[i]) != 0 ||
                mCwbBuffers[i] == nullptr) {
                mInitFailures++;
                LOG(WARNING) << "Failed to allocate CWB buffer " << i << " (attempt "
                             << mInitFailures << ")";
                releaseCwb();
                return false;
            }
            AHardwareBuffer_describe(mCwbBuffers[i], &desc);
            mCwbBufferHandles[i] = AHardwareBuffer_getNativeHandle(mCwbBuffers[i]);
            mBufferStride = desc.stride;
        }
    }

    mInitFailures = 0;
    LOG(INFO) << "CwbClient initialized hardware CWB double-buffer: " << mRoiWidth << "x"
              << mRoiHeight << " RGBA, stride=" << mBufferStride;
    return true;
}

void CwbClient::releaseCwb() {
    if (mCwbClient != nullptr) {
        DisplayConfig::ClientInterface::Destroy(mCwbClient);
        mCwbClient = nullptr;
    }

    for (size_t i = 0; i < kBufferCount; ++i) {
        if (mCwbBuffers[i] != nullptr) {
            AHardwareBuffer_release(mCwbBuffers[i]);
            mCwbBuffers[i] = nullptr;
        }
        mCwbBufferHandles[i] = nullptr;
    }
}

bool CwbClient::start() {
    std::lock_guard<std::mutex> lock(mLock);
    if (mCallbackHandler != nullptr) {
        mCallbackHandler->forceReset();
    }
    mInitFailures = 0;
    mPendingTimeouts = 0;
    mRefreshAttempts = 0;
    mPaused = false;

    if (!mWorker.joinable()) {
        mRunning = true;
        mWorker = std::thread(&CwbClient::captureWorkerThread, this);
    } else {
        mCv.notify_all();
    }
    return true;
}

void CwbClient::stop() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mPaused = true;
    }

    if (mCallbackHandler != nullptr) {
        mCallbackHandler->abort();
        if (mCallbackHandler->isPending()) {
            mCallbackHandler->waitForDrain(std::chrono::milliseconds(80));
        }
        mCallbackHandler->forceReset();
    }

    mCv.notify_all();
    mPendingTimeouts = 0;
    mRefreshAttempts = 0;
}

bool CwbClient::requestCwbSample() {
    if (!mRunning || mPaused || mCompensator == nullptr || mCompensator->getBacklight() == 0) {
        return false;
    }

    if (!ensureCwbInitialized()) {
        return false;
    }

    size_t submitIdx = mCurrentBufferIndex;
    const struct native_handle* submitHandle = mCwbBufferHandles[submitIdx];
    if (submitHandle == nullptr) {
        return false;
    }

    int32_t cwbError = 0;
    bool bufferReady = mCallbackHandler->consumeIfDone(&cwbError);

    if (!bufferReady && mCallbackHandler->isPending()) {
        if (mCallbackHandler->waitForDone(std::chrono::milliseconds(20), &cwbError)) {
            bufferReady = true;
            mPendingTimeouts = 0;
        } else {
            if (++mPendingTimeouts < 5) {
                if (mRunning && !mPaused && !mCompensator->hasSample() && mRefreshAttempts < 3) {
                    mCwbClient->RefreshScreen();
                    mRefreshAttempts++;
                }
            } else {
                mCallbackHandler->forceReset();
                mPendingTimeouts = 0;
            }
            return false;
        }
    }

    if (!bufferReady) {
        mPendingTimeouts = 0;
        DisplayConfig::Rect roiRect = {
                .left = mRoi.left,
                .top = mRoi.top,
                .right = mRoi.right,
                .bottom = mRoi.bottom,
        };

        mCallbackHandler->prepareForCapture(mNextRequestId++);

        int ret = mCwbClient->SetCWBOutputBuffer(kPrimaryDisplayId, roiRect, true, submitHandle);
        if (ret != 0) {
            mCallbackHandler->cancelPending();
            return false;
        }

        if (mRunning && !mPaused && !mCompensator->hasSample() && mRefreshAttempts < 3) {
            mCwbClient->RefreshScreen();
            mRefreshAttempts++;
        }

        if (mCallbackHandler->waitForDone(std::chrono::milliseconds(80), &cwbError)) {
            bufferReady = true;
        } else {
            return false;
        }
    }

    if (cwbError != 0) {
        return false;
    }
    mPendingTimeouts = 0;

    AHardwareBuffer* currentBuffer = mCwbBuffers[submitIdx];
    ScopedHardwareBufferLock scopedLock(currentBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN);
    if (!scopedLock.isLocked()) {
        return false;
    }

    const uint8_t* pixels = static_cast<const uint8_t*>(scopedLock.address());
    float weightedR = 0.0f;
    float weightedG = 0.0f;
    float weightedB = 0.0f;
    float totalWeight = 0.0f;

    for (int32_t y = 0; y < mRoiHeight; ++y) {
        int32_t dy = y - mCenterRelY;
        int32_t dy2 = dy * dy;
        const uint8_t* row = pixels + (static_cast<size_t>(y) * mBufferStride * 4);

        for (int32_t x = 0; x < mRoiWidth; ++x) {
            int32_t dx = x - mCenterRelX;
            int32_t distSq = dx * dx + dy2;

            if (distSq > mRadius3Sq) {
                continue;
            }

            float w;
            if (distSq <= mRadius1Sq) {
                w = 1.0f;
            } else if (distSq <= mRadius2Sq) {
                w = 0.50f;
            } else {
                w = 0.20f;
            }

            int32_t offset = x * 4;
            uint8_t r = row[offset];
            uint8_t g = row[offset + 1];
            uint8_t b = row[offset + 2];

            weightedR += kGammaLut[r] * w;
            weightedG += kGammaLut[g] * w;
            weightedB += kGammaLut[b] * w;
            totalWeight += w;
        }
    }

    // Toggle double-buffer index for the next capture
    mCurrentBufferIndex = (submitIdx + 1) % kBufferCount;

    if (totalWeight > 0.0f) {
        float normR = std::clamp(weightedR / totalWeight, 0.0f, 1.0f);
        float normG = std::clamp(weightedG / totalWeight, 0.0f, 1.0f);
        float normB = std::clamp(weightedB / totalWeight, 0.0f, 1.0f);
        mCompensator->setSampledRgb(normR, normG, normB);
        mRefreshAttempts = 0;
        return true;
    }

    return false;
}

void CwbClient::captureWorkerThread() {
    pthread_setname_np(pthread_self(), "UDalsCwbWorker");

    const auto kSampleInterval = std::chrono::milliseconds(40);

    while (mRunning) {
        {
            std::unique_lock<std::mutex> lock(mLock);
            mCv.wait(lock, [this] { return !mRunning || !mPaused; });
            if (!mRunning) {
                break;
            }
        }

        bool sampled = requestCwbSample();

        auto sleepDuration = kSampleInterval;
        if (mInitFailures >= 15) {
            sleepDuration = std::chrono::seconds(5);
        } else if (mCompensator != nullptr && mCompensator->getBacklight() == 0) {
            sleepDuration = std::chrono::milliseconds(200);
        } else if (!sampled && mPendingTimeouts > 0) {
            sleepDuration = std::chrono::milliseconds(150);
        }

        std::unique_lock<std::mutex> lock(mLock);
        mCv.wait_for(lock, sleepDuration, [this] { return !mRunning || mPaused; });
    }
}
