/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsCwbClient"

#include "CwbClient.h"

#include <android-base/logging.h>
#include <android/hardware_buffer.h>
#include <config/client_interface.h>
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

namespace {

constexpr uint32_t kPrimaryDisplayId = static_cast<uint32_t>(DisplayConfig::DisplayType::kPrimary);

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
            if (err == 0) {
                if (mAddr != nullptr) {
                    mLocked = true;
                } else {
                    LOG(ERROR) << "AHardwareBuffer_lock succeeded with null address";
                    AHardwareBuffer_unlock(mBuffer, nullptr);
                }
            }
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
        if (!mPending) {
            return;
        }
        mError = error;
        mDone = true;
        mPending = false;
        mCv.notify_all();
    }

    bool prepareForCapture(uint64_t gen) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mAbortedGen > gen) {
            mAborted = true;
            return false;
        }
        mPostedGen = gen;
        mActiveGen = std::max(mActiveGen, gen);
        mDone = false;
        mAborted = false;
        mError = 0;
        mPending = true;
        return true;
    }

    void cancelPending() {
        std::lock_guard<std::mutex> lock(mMutex);
        mPending = false;
    }

    bool isPending() const {
        std::lock_guard<std::mutex> lock(mMutex);
        return mPending;
    }

    uint64_t postedGen() const {
        std::lock_guard<std::mutex> lock(mMutex);
        return mPostedGen;
    }

    bool consumeIfDone(int32_t* outError, uint64_t* outPostedGen) {
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mDone) {
            return false;
        }
        *outError = mError;
        *outPostedGen = mPostedGen;
        mDone = false;
        return true;
    }

    bool waitForDone(std::chrono::milliseconds timeout, int32_t* outError, uint64_t* outPostedGen) {
        std::unique_lock<std::mutex> lock(mMutex);
        if (!mCv.wait_for(lock, timeout, [this] { return mDone || mAborted; })) {
            return false;
        }
        if (mAborted) {
            return false;
        }
        *outError = mError;
        *outPostedGen = mPostedGen;
        mDone = false;
        return true;
    }

    void abort(uint64_t gen) {
        std::lock_guard<std::mutex> lock(mMutex);
        mAbortedGen = std::max(mAbortedGen, gen);
        if (mAbortedGen > mActiveGen) {
            mAborted = true;
            mCv.notify_all();
        }
    }

    void clearAbortedIfBefore(uint64_t activeGen) {
        std::lock_guard<std::mutex> lock(mMutex);
        mActiveGen = std::max(mActiveGen, activeGen);
        if (mAbortedGen <= mActiveGen) {
            mAborted = false;
        }
    }

    bool waitForDrain(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mMutex);
        mCv.wait_for(lock, timeout, [this] { return !mPending || mAborted; });
        return !mPending;
    }

  private:
    mutable std::mutex mMutex;
    std::condition_variable mCv;
    uint64_t mPostedGen = 0;
    uint64_t mActiveGen = 0;
    uint64_t mAbortedGen = 0;
    bool mDone = false;
    bool mPending = false;
    bool mAborted = false;
    int32_t mError = 0;
};

CwbClient::CwbClient(UdalsGeometry geo, std::shared_ptr<UDalsBleedCompensator> compensator)
    : mGeo(geo),
      mRoiWidth(geo.right - geo.left),
      mRoiHeight(geo.bottom - geo.top),
      mCenterRelX(geo.cx - geo.left),
      mCenterRelY(geo.cy - geo.top),
      mCompensator(std::move(compensator)) {
    mRadius1Sq = geo.r1 * geo.r1;
    mRadius2Sq = geo.r2 * geo.r2;
    mRadius3Sq = geo.r3 * geo.r3;
}

CwbClient::~CwbClient() {
    std::shared_ptr<CwbCallbackHandler> cb;
    uint64_t gen = 0;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        mRunning = false;
        mPaused = true;
        gen = ++mCaptureGen;
        cb = mActiveCallback;
    }

    if (cb != nullptr) {
        cb->abort(gen);
    }

    mCv.notify_all();

    if (mWorker.joinable()) {
        mWorker.join();
    }

    releaseCwbWorker(/*freeBuffer=*/true);
    mRetiredCallbacks.clear();
}

bool CwbClient::hasActiveBacklight() const {
    return mCompensator == nullptr || mCompensator->getBacklight() > 0;
}

bool CwbClient::isActiveGen(uint64_t gen) const {
    std::lock_guard<std::mutex> lock(mControlMutex);
    return mRunning && !mPaused && mCaptureGen == gen;
}

void CwbClient::notifyBacklightChanged() {
    std::lock_guard<std::mutex> lock(mControlMutex);
    mCv.notify_all();
}

void CwbClient::retireCallbackHandlerWorker(std::shared_ptr<CwbCallbackHandler> handler) {
    if (handler == nullptr) {
        return;
    }
    // Bounded retire queue: guards against a late NotifyCWBBufferDone firing
    // after Destroy() (vendor DisplayConfig destructor races with in-flight
    // callbacks). Assumes >8 simultaneous in-flight callbacks is impossible;
    // each drains within ~80 ms in practice.
    constexpr size_t kMaxRetiredCallbacks = 8;
    mRetiredCallbacks.push_back(std::move(handler));
    while (mRetiredCallbacks.size() > kMaxRetiredCallbacks) {
        mRetiredCallbacks.pop_front();
    }
}

void CwbClient::triggerRefreshWorker() {
    if (mCwbClient != nullptr) {
        mCwbClient->RefreshScreen();
    }
}

bool CwbClient::ensureCwbInitializedWorker() {
    if (mInvalidRoi.load() || mGeo.left < 0 || mGeo.top < 0 || mRoiWidth <= 0 || mRoiHeight <= 0) {
        return false;
    }

    if (mCwbClient != nullptr && mCwbBufferHandle != nullptr) {
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
        auto callbackHandler = std::make_shared<CwbCallbackHandler>();
        DisplayConfig::ClientInterface* client = nullptr;
        if (DisplayConfig::ClientInterface::Create("sensor-notifier", callbackHandler.get(),
                                                   &client) != 0 ||
            client == nullptr) {
            uint32_t attempt = ++mInitFailures;
            LOG(WARNING) << "DisplayConfig ClientInterface Create failed (attempt " << attempt
                         << ")";
            releaseCwbWorker(false);
            return false;
        }
        mCallbackHandler = std::move(callbackHandler);
        {
            std::lock_guard<std::mutex> lock(mControlMutex);
            mActiveCallback = mCallbackHandler;
        }
        mCwbClient = client;
    }

    if (!mDisplayBoundsChecked) {
        DisplayConfig::Attributes attrs{};
        if (mCwbClient->GetActiveBuiltinDisplayAttributes(&attrs) != 0 || attrs.x_res == 0 ||
            attrs.y_res == 0) {
            uint32_t attempt = ++mInitFailures;
            LOG(WARNING) << "Failed to query active builtin display attributes (attempt " << attempt
                         << ")";
            releaseCwbWorker(true);
            return false;
        }
        mDisplayBoundsChecked = true;
        if (static_cast<uint64_t>(mGeo.right) > attrs.x_res ||
            static_cast<uint64_t>(mGeo.bottom) > attrs.y_res) {
            LOG(ERROR) << "UDALS CWB ROI (" << mGeo.left << "," << mGeo.top << "," << mGeo.right
                       << "," << mGeo.bottom << ") exceeds active display resolution "
                       << attrs.x_res << "x" << attrs.y_res << "; disabling CWB";
            mInvalidRoi = true;
            releaseCwbWorker(true);
            return false;
        }
    }

    if (mCwbBuffer == nullptr || mCwbBufferHandle == nullptr) {
        AHardwareBuffer_Desc desc = {
                .width = static_cast<uint32_t>(mRoiWidth),
                .height = static_cast<uint32_t>(mRoiHeight),
                .layers = 1,
                .format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM,
                .usage = AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN |
                         AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT |
                         AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY,
                .stride = 0,
                .rfu0 = 0,
                .rfu1 = 0,
        };

        if (AHardwareBuffer_allocate(&desc, &mCwbBuffer) != 0 || mCwbBuffer == nullptr) {
            uint32_t attempt = ++mInitFailures;
            LOG(WARNING) << "Failed to allocate CWB buffer (attempt " << attempt << ")";
            releaseCwbWorker(true);
            return false;
        }

        AHardwareBuffer_describe(mCwbBuffer, &desc);
        mBufferStride = desc.stride;
        if (mBufferStride < static_cast<uint32_t>(mRoiWidth) ||
            desc.height < static_cast<uint32_t>(mRoiHeight)) {
            mInitFailures++;
            LOG(ERROR) << "Allocated CWB buffer smaller than ROI";
            releaseCwbWorker(true);
            return false;
        }

        mCwbBufferHandle = AHardwareBuffer_getNativeHandle(mCwbBuffer);
        if (mCwbBufferHandle == nullptr) {
            mInitFailures++;
            LOG(WARNING) << "Failed to get native handle for CWB buffer";
            releaseCwbWorker(true);
            return false;
        }

        LOG(INFO) << "CWB client initialized successfully: " << mRoiWidth << "x" << mRoiHeight
                  << " stride=" << mBufferStride;
    }

    return true;
}

void CwbClient::releaseCwbWorker(bool freeBuffer) {
    uint64_t abortGen = 0;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        abortGen = mCaptureGen;
        mActiveCallback.reset();
    }

    std::shared_ptr<CwbCallbackHandler> retired = std::move(mCallbackHandler);
    if (retired != nullptr) {
        retired->abort(abortGen);
        retired->cancelPending();
        retireCallbackHandlerWorker(std::move(retired));
    }

    if (mCwbClient != nullptr) {
        DisplayConfig::ClientInterface::Destroy(mCwbClient);
        mCwbClient = nullptr;
    }

    if (freeBuffer) {
        if (mCwbBuffer != nullptr) {
            AHardwareBuffer_release(mCwbBuffer);
            mCwbBuffer = nullptr;
            mCwbBufferHandle = nullptr;
        }
        mBufferStride = 0;
    }
}

bool CwbClient::start() {
    if (mInvalidRoi.load()) {
        return false;
    }
    std::shared_ptr<CwbCallbackHandler> cb;
    uint64_t gen = 0;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        gen = ++mCaptureGen;
        mPaused = false;
        cb = mActiveCallback;
        if (!mRunning) {
            mRunning = true;
            mWorker = std::thread(&CwbClient::captureWorkerThread, this);
        }
    }
    if (cb != nullptr) {
        cb->clearAbortedIfBefore(gen);
    }
    mCv.notify_all();
    return true;
}

void CwbClient::stop() {
    std::shared_ptr<CwbCallbackHandler> cb;
    uint64_t gen = 0;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        if (!mRunning || mPaused) {
            return;
        }
        mPaused = true;
        gen = ++mCaptureGen;
        cb = mActiveCallback;
    }

    if (cb != nullptr) {
        cb->abort(gen);
    }
    mCv.notify_all();
}

void CwbClient::computeWeightedRgb(const uint8_t* pixels, float* outR, float* outG, float* outB,
                                   float* outTotalWeight) const {
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

    *outR = weightedR;
    *outG = weightedG;
    *outB = weightedB;
    *outTotalWeight = totalWeight;
}

bool CwbClient::requestCwbSample(bool* outRefreshTriggered) {
    if (outRefreshTriggered != nullptr) {
        *outRefreshTriggered = false;
    }

    uint64_t currentGen = 0;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        if (!mRunning || mPaused || mCompensator == nullptr) {
            return false;
        }
        currentGen = mCaptureGen;
    }

    if (currentGen != mLastSeenGen) {
        mInitFailures = 0;
        mPendingTimeouts = 0;
        mConsecutiveErrors = 0;
        mRefreshAttempts = 0;
        mLastSeenGen = currentGen;
    }

    if (mCompensator->getBacklight() == 0) {
        return false;
    }

    int32_t cwbError = 0;
    uint64_t postedGen = 0;
    bool bufferReady = false;
    bool wasPending = false;

    if (mCallbackHandler != nullptr) {
        if (mCallbackHandler->consumeIfDone(&cwbError, &postedGen)) {
            if (postedGen == currentGen) {
                bufferReady = true;
            }
        }

        if (!bufferReady && mCallbackHandler->isPending()) {
            if (mCallbackHandler->postedGen() != currentGen) {
                // Drain in-flight pre-sleep capture across off->on wake before dispatching anew.
                mCallbackHandler->clearAbortedIfBefore(currentGen);
                triggerRefreshWorker();
                bool drained = mCallbackHandler->waitForDrain(std::chrono::milliseconds(150));
                if (!isActiveGen(currentGen)) {
                    return false;
                }
                if (drained) {
                    int32_t ignoredErr = 0;
                    uint64_t ignoredGen = 0;
                    mCallbackHandler->consumeIfDone(&ignoredErr, &ignoredGen);
                    mPendingTimeouts = 0;
                    mRefreshAttempts = 0;
                } else {
                    releaseCwbWorker(/*freeBuffer=*/true);
                    mInitFailures++;
                    mPendingTimeouts = 0;
                    mRefreshAttempts = 0;
                    return false;
                }
            } else {
                wasPending = true;
            }
        }
    }

    if (!bufferReady && !wasPending) {
        if (!ensureCwbInitializedWorker() || mCwbBufferHandle == nullptr ||
            mCallbackHandler == nullptr) {
            return false;
        }

        DisplayConfig::Rect roiRect = {
                .left = static_cast<uint32_t>(mGeo.left),
                .top = static_cast<uint32_t>(mGeo.top),
                .right = static_cast<uint32_t>(mGeo.right),
                .bottom = static_cast<uint32_t>(mGeo.bottom),
        };

        if (!isActiveGen(currentGen) || !mCallbackHandler->prepareForCapture(currentGen)) {
            return false;
        }
        int ret =
                mCwbClient->SetCWBOutputBuffer(kPrimaryDisplayId, roiRect, true, mCwbBufferHandle);
        if (ret != 0) {
            mCallbackHandler->cancelPending();
            ++mConsecutiveErrors;
            return false;
        }
    }

    if (!bufferReady) {
        auto waitMs = wasPending ? std::chrono::milliseconds(20) : std::chrono::milliseconds(80);
        if (mCallbackHandler->waitForDone(waitMs, &cwbError, &postedGen)) {
            if (postedGen != currentGen) {
                return false;
            }
            bufferReady = true;
        } else {
            if (!isActiveGen(currentGen) || mCompensator == nullptr) {
                return false;
            }
            mPendingTimeouts++;
            if (!mCompensator->hasSample() && mRefreshAttempts < 3) {
                triggerRefreshWorker();
                mRefreshAttempts++;
                if (outRefreshTriggered != nullptr) {
                    *outRefreshTriggered = true;
                }
            }
            return false;
        }
    }

    if (!isActiveGen(currentGen) || mCompensator == nullptr || mCwbBuffer == nullptr) {
        return false;
    }

    mPendingTimeouts = 0;

    if (cwbError != 0) {
        ++mConsecutiveErrors;
        return false;
    }

    ScopedHardwareBufferLock scopedLock(mCwbBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN);
    if (!scopedLock.isLocked()) {
        ++mConsecutiveErrors;
        return false;
    }

    mConsecutiveErrors = 0;
    mInitFailures = 0;

    float weightedR = 0.0f;
    float weightedG = 0.0f;
    float weightedB = 0.0f;
    float totalWeight = 0.0f;
    computeWeightedRgb(static_cast<const uint8_t*>(scopedLock.address()), &weightedR, &weightedG,
                       &weightedB, &totalWeight);

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

    // 5 FPS (200 ms) limits CWB frequency boost while matching ALS integration periods
    const auto kSampleInterval = std::chrono::milliseconds(200);

    while (true) {
        {
            std::unique_lock<std::mutex> lock(mControlMutex);
            mCv.wait(lock, [this] {
                return !mRunning || (!mPaused && !mInvalidRoi.load() && hasActiveBacklight());
            });
            if (!mRunning) {
                break;
            }
        }

        bool refreshTriggered = false;
        bool sampled = requestCwbSample(&refreshTriggered);

        if (mInvalidRoi.load()) {
            continue;
        }

        bool shouldSkipSleep = false;
        {
            std::lock_guard<std::mutex> lock(mControlMutex);
            if (!mRunning) {
                break;
            }
            if (!mPaused && !hasActiveBacklight()) {
                shouldSkipSleep = true;
            }
        }
        if (shouldSkipSleep) {
            continue;
        }

        auto sleepDuration = kSampleInterval;
        if (mInitFailures >= 15 || mConsecutiveErrors >= 15) {
            sleepDuration = std::chrono::seconds(5);
        } else if (refreshTriggered) {
            // Re-check within ~2 vsyncs so initial wake compensation beats the wake flare gate.
            sleepDuration = std::chrono::milliseconds(30);
        } else if (!sampled &&
                   (mPendingTimeouts > 0 || mConsecutiveErrors > 0 || mInitFailures > 0)) {
            sleepDuration = (mPendingTimeouts >= 5 || mConsecutiveErrors >= 5 || mInitFailures >= 5)
                                    ? std::chrono::milliseconds(500)
                                    : std::chrono::milliseconds(300);
        }

        std::unique_lock<std::mutex> lock(mControlMutex);
        mCv.wait_for(lock, sleepDuration, [this] { return !mRunning || mPaused; });
        if (!mRunning) {
            break;
        }
    }
}
