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
constexpr float kMaxBleedRelevantLux = 15000.0f;
constexpr float kMinTriggerDeltaLux = 1.5f;
constexpr float kSustainedMinTriggerDeltaLux = 0.40f;
constexpr uint32_t kSustainedLowLuxSamples = 2u;
constexpr float kMaxTriggerDeltaLux = 250.0f;
constexpr float kTriggerRelativeDelta = 0.08f;
constexpr auto kMinDispatchInterval = std::chrono::milliseconds(250);
constexpr auto kInitialWaitTimeout = std::chrono::milliseconds(100);
constexpr auto kPokeWaitTimeout = std::chrono::milliseconds(100);
constexpr auto kDrainTimeout = std::chrono::milliseconds(150);
constexpr auto kLockoutProbeInterval = std::chrono::seconds(2);
constexpr uint32_t kLockoutThreshold = 3u;

constexpr float kGammaHigh = 2.0f;
constexpr float kGammaLow = 1.6f;

std::array<float, 256> initGammaLut(float gamma) {
    std::array<float, 256> lut{};
    for (size_t i = 0; i < 256; ++i) {
        lut[i] = std::pow(static_cast<float>(i) / 255.0f, gamma);
    }
    return lut;
}

const std::array<float, 256> kGammaLutHigh = initGammaLut(kGammaHigh);
const std::array<float, 256> kGammaLutLow = initGammaLut(kGammaLow);

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
    if (mRunning && !mPaused && !mAboveBleedRelevantLux && (mCaptureRequested || mInLockout) &&
        hasActiveBacklight()) {
        mCv.notify_all();
    }
}

void CwbClient::onRawLuxSample(float rawLux) {
    if (!std::isfinite(rawLux) || rawLux < 0.0f || mInvalidRoi.load()) {
        return;
    }

    uint64_t seqBefore = (mCompensator != nullptr) ? mCompensator->getSampleSeq() : 0;
    bool shouldNotify = false;
    bool upwardPending = false;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        if (!mRunning || mPaused) {
            return;
        }
        if (rawLux >= kMaxBleedRelevantLux) {
            bool changed = !mAboveBleedRelevantLux;
            mAboveBleedRelevantLux = true;
            mHasLatestRawLux = false;
            mHasRawLuxAnchor = false;
            mSeedAnchorOnNextSample = false;
            mSustainedDeltaCount = 0;
            mSustainedDeltaSign = 0;
            mUpwardCapturePending = false;
            mCaptureRequested = false;
            shouldNotify = changed;
        } else {
            bool resumedFromHighLux = mAboveBleedRelevantLux;
            mAboveBleedRelevantLux = false;
            mLatestRawLux = rawLux;
            mHasLatestRawLux = true;
            if (mInLockout) {
                mSustainedDeltaCount = 0;
                mSustainedDeltaSign = 0;
                mUpwardCapturePending = false;
                shouldNotify = resumedFromHighLux;
            } else if (mSeedAnchorOnNextSample) {
                mLastTriggerRawLux = rawLux;
                mHasRawLuxAnchor = true;
                mSeedAnchorOnNextSample = false;
                mSustainedDeltaCount = 0;
                mSustainedDeltaSign = 0;
                mUpwardCapturePending = false;
            } else {
                float delta = rawLux - mLastTriggerRawLux;
                float absDelta = std::fabs(delta);
                float immediateThreshold = std::clamp(kTriggerRelativeDelta * mLastTriggerRawLux,
                                                      kMinTriggerDeltaLux, kMaxTriggerDeltaLux);
                float sustainedThreshold =
                        std::clamp(kTriggerRelativeDelta * mLastTriggerRawLux,
                                   kSustainedMinTriggerDeltaLux, kMaxTriggerDeltaLux);
                bool shouldTrigger = false;
                if (!mHasRawLuxAnchor) {
                    mSustainedDeltaCount = 0;
                    mSustainedDeltaSign = 0;
                    mUpwardCapturePending = false;
                    shouldTrigger = true;
                } else if (absDelta >= immediateThreshold) {
                    mSustainedDeltaCount = 0;
                    mSustainedDeltaSign = 0;
                    mUpwardCapturePending = (delta > 0.0f);
                    shouldTrigger = true;
                } else if (absDelta >= sustainedThreshold) {
                    int8_t sign = (delta > 0.0f) ? 1 : -1;
                    if (sign == mSustainedDeltaSign) {
                        mSustainedDeltaCount++;
                    } else {
                        mSustainedDeltaSign = sign;
                        mSustainedDeltaCount = 1;
                    }
                    if (sign < 0) {
                        mUpwardCapturePending = false;
                    }
                    if (mSustainedDeltaCount >= kSustainedLowLuxSamples) {
                        mSustainedDeltaCount = 0;
                        mSustainedDeltaSign = 0;
                        mUpwardCapturePending = (sign > 0);
                        shouldTrigger = true;
                    }
                } else {
                    mSustainedDeltaCount = 0;
                    mSustainedDeltaSign = 0;
                }

                if (shouldTrigger && !mCaptureRequested) {
                    mCaptureRequested = true;
                    shouldNotify = true;
                }

                upwardPending = hasActiveBacklight() &&
                                (mUpwardCapturePending ||
                                 (mSustainedDeltaCount > 0 && mSustainedDeltaSign > 0));
            }
        }
    }
    if (mCompensator != nullptr) {
        mCompensator->updateTransitionHold(upwardPending, rawLux, seqBefore);
    }
    if (shouldNotify) {
        mCv.notify_one();
    }
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
    if (mInitFailures >= 15 && std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now - mLastInitAttempt) < std::chrono::seconds(5)) {
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
        mInLockout = false;
        mAboveBleedRelevantLux = false;
        mHasLatestRawLux = false;
        mLatestRawLux = 0.0f;
        mHasRawLuxAnchor = false;
        mLastTriggerRawLux = 0.0f;
        mSeedAnchorOnNextSample = false;
        mSustainedDeltaCount = 0;
        mSustainedDeltaSign = 0;
        mUpwardCapturePending = false;
        mCaptureRequested = true;
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
        mCaptureRequested = false;
        mInLockout = false;
        mAboveBleedRelevantLux = false;
        mHasLatestRawLux = false;
        mLatestRawLux = 0.0f;
        mHasRawLuxAnchor = false;
        mLastTriggerRawLux = 0.0f;
        mSeedAnchorOnNextSample = false;
        mSustainedDeltaCount = 0;
        mSustainedDeltaSign = 0;
        mUpwardCapturePending = false;
        gen = ++mCaptureGen;
        cb = mActiveCallback;
    }

    if (cb != nullptr) {
        cb->abort(gen);
    }
    if (mCompensator != nullptr) {
        mCompensator->clearTransitionHold();
    }
    mCv.notify_all();
}

void CwbClient::computeWeightedRgb(const uint8_t* pixels, float* outRHigh, float* outGHigh,
                                   float* outBHigh, float* outRLow, float* outGLow, float* outBLow,
                                   float* outTotalWeight) const {
    float weightedRHigh = 0.0f;
    float weightedGHigh = 0.0f;
    float weightedBHigh = 0.0f;
    float weightedRLow = 0.0f;
    float weightedGLow = 0.0f;
    float weightedBLow = 0.0f;
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

            weightedRHigh += kGammaLutHigh[r] * w;
            weightedGHigh += kGammaLutHigh[g] * w;
            weightedBHigh += kGammaLutHigh[b] * w;
            weightedRLow += kGammaLutLow[r] * w;
            weightedGLow += kGammaLutLow[g] * w;
            weightedBLow += kGammaLutLow[b] * w;
            totalWeight += w;
        }
    }

    *outRHigh = weightedRHigh;
    *outGHigh = weightedGHigh;
    *outBHigh = weightedBHigh;
    *outRLow = weightedRLow;
    *outGLow = weightedGLow;
    *outBLow = weightedBLow;
    *outTotalWeight = totalWeight;
}

CwbClient::CaptureResult CwbClient::requestCwbSample(uint64_t currentGen) {
    bool hadDispatchLux = false;
    float dispatchRawLux = 0.0f;
    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        if (!mRunning || mPaused || mCaptureGen != currentGen || mCompensator == nullptr ||
            mAboveBleedRelevantLux || !hasActiveBacklight()) {
            return CaptureResult::kAborted;
        }
        hadDispatchLux = mHasLatestRawLux;
        dispatchRawLux = mLatestRawLux;
        mSustainedDeltaCount = 0;
        mSustainedDeltaSign = 0;
        mCaptureRequested = false;
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
                bool drained = mCallbackHandler->waitForDrain(kDrainTimeout);
                if (!isActiveGen(currentGen)) {
                    return CaptureResult::kAborted;
                }
                if (drained) {
                    int32_t ignoredErr = 0;
                    uint64_t ignoredGen = 0;
                    mCallbackHandler->consumeIfDone(&ignoredErr, &ignoredGen);
                } else {
                    releaseCwbWorker(/*freeBuffer=*/true);
                    return CaptureResult::kFailed;
                }
            } else {
                wasPending = true;
            }
        }
    }

    if (!bufferReady && !wasPending) {
        if (!ensureCwbInitializedWorker() || mCwbBufferHandle == nullptr ||
            mCallbackHandler == nullptr) {
            return isActiveGen(currentGen) ? CaptureResult::kFailed : CaptureResult::kAborted;
        }

        DisplayConfig::Rect roiRect = {
                .left = static_cast<uint32_t>(mGeo.left),
                .top = static_cast<uint32_t>(mGeo.top),
                .right = static_cast<uint32_t>(mGeo.right),
                .bottom = static_cast<uint32_t>(mGeo.bottom),
        };

        if (!isActiveGen(currentGen) || !mCallbackHandler->prepareForCapture(currentGen)) {
            return CaptureResult::kAborted;
        }
        mLastDispatchTime = std::chrono::steady_clock::now();
        mHasDispatchedCapture = true;

        int ret =
                mCwbClient->SetCWBOutputBuffer(kPrimaryDisplayId, roiRect, true, mCwbBufferHandle);
        if (ret != 0) {
            mCallbackHandler->cancelPending();
            return isActiveGen(currentGen) ? CaptureResult::kFailed : CaptureResult::kAborted;
        }
    }

    if (!bufferReady) {
        if (!mCallbackHandler->waitForDone(kInitialWaitTimeout, &cwbError, &postedGen)) {
            if (!isActiveGen(currentGen) || mCompensator == nullptr) {
                return CaptureResult::kAborted;
            }
            if (wasPending) {
                return CaptureResult::kFailed;
            }
            // Idle SurfaceFlinger may defer CWB completion until a commit occurs:
            // poke RefreshScreen once and wait immediately without dispatch cooldown.
            triggerRefreshWorker();
            if (!mCallbackHandler->waitForDone(kPokeWaitTimeout, &cwbError, &postedGen)) {
                return isActiveGen(currentGen) ? CaptureResult::kFailed : CaptureResult::kAborted;
            }
        }
        if (postedGen != currentGen) {
            return CaptureResult::kAborted;
        }
    }

    if (!isActiveGen(currentGen) || mCompensator == nullptr || mCwbBuffer == nullptr) {
        return CaptureResult::kAborted;
    }

    if (cwbError != 0) {
        return CaptureResult::kFailed;
    }

    ScopedHardwareBufferLock scopedLock(mCwbBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN);
    if (!scopedLock.isLocked()) {
        return CaptureResult::kFailed;
    }

    float weightedRHigh = 0.0f;
    float weightedGHigh = 0.0f;
    float weightedBHigh = 0.0f;
    float weightedRLow = 0.0f;
    float weightedGLow = 0.0f;
    float weightedBLow = 0.0f;
    float totalWeight = 0.0f;
    computeWeightedRgb(static_cast<const uint8_t*>(scopedLock.address()), &weightedRHigh,
                       &weightedGHigh, &weightedBHigh, &weightedRLow, &weightedGLow, &weightedBLow,
                       &totalWeight);

    if (totalWeight <= 0.0f) {
        return CaptureResult::kFailed;
    }

    float normRHigh = std::clamp(weightedRHigh / totalWeight, 0.0f, 1.0f);
    float normGHigh = std::clamp(weightedGHigh / totalWeight, 0.0f, 1.0f);
    float normBHigh = std::clamp(weightedBHigh / totalWeight, 0.0f, 1.0f);
    float normRLow = std::clamp(weightedRLow / totalWeight, 0.0f, 1.0f);
    float normGLow = std::clamp(weightedGLow / totalWeight, 0.0f, 1.0f);
    float normBLow = std::clamp(weightedBLow / totalWeight, 0.0f, 1.0f);
    mCompensator->setSampledRgb(normRHigh, normGHigh, normBHigh, normRLow, normGLow, normBLow);

    // Transition HWC kCWBConfigure -> kCWBTeardown after a 1-shot CWB capture so idle
    // SurfaceFlinger does not remain in active CWB state until the next UI redraw.
    triggerRefreshWorker();

    mConsecutiveErrors = 0;
    mPendingStuckCount = 0;
    mInitFailures = 0;
    mRetryAfter = {};

    {
        std::lock_guard<std::mutex> lock(mControlMutex);
        if (!mRunning || mPaused || mCaptureGen != currentGen) {
            return CaptureResult::kAborted;
        }
        mInLockout = false;
        if (hadDispatchLux) {
            if (mHasLatestRawLux) {
                mLastTriggerRawLux = dispatchRawLux;
                mHasRawLuxAnchor = true;
                mSeedAnchorOnNextSample = false;
                float delta = mLatestRawLux - mLastTriggerRawLux;
                float absDelta = std::fabs(delta);
                float immediateThreshold = std::clamp(kTriggerRelativeDelta * mLastTriggerRawLux,
                                                      kMinTriggerDeltaLux, kMaxTriggerDeltaLux);
                float sustainedThreshold =
                        std::clamp(kTriggerRelativeDelta * mLastTriggerRawLux,
                                   kSustainedMinTriggerDeltaLux, kMaxTriggerDeltaLux);
                if (absDelta >= immediateThreshold) {
                    mCaptureRequested = true;
                    mUpwardCapturePending = (delta > 0.0f);
                    mSustainedDeltaCount = 0;
                    mSustainedDeltaSign = 0;
                } else if (absDelta >= sustainedThreshold) {
                    mCaptureRequested = false;
                    mUpwardCapturePending = false;
                    mSustainedDeltaSign = (delta > 0.0f) ? 1 : -1;
                    mSustainedDeltaCount = 1;
                } else {
                    mCaptureRequested = false;
                    mUpwardCapturePending = false;
                    mSustainedDeltaCount = 0;
                    mSustainedDeltaSign = 0;
                }
            } else {
                // Ambient lux crossed >= kMaxBleedRelevantLux mid-capture.
                mHasRawLuxAnchor = false;
                mSeedAnchorOnNextSample = false;
                mSustainedDeltaCount = 0;
                mSustainedDeltaSign = 0;
                mUpwardCapturePending = false;
                mCaptureRequested = false;
            }
        } else if (mHasLatestRawLux) {
            mLastTriggerRawLux = mLatestRawLux;
            mHasRawLuxAnchor = true;
            mSeedAnchorOnNextSample = false;
            mSustainedDeltaCount = 0;
            mSustainedDeltaSign = 0;
            mUpwardCapturePending = false;
            mCaptureRequested = false;
        } else {
            mHasRawLuxAnchor = false;
            mSeedAnchorOnNextSample = true;
            mSustainedDeltaCount = 0;
            mSustainedDeltaSign = 0;
            mUpwardCapturePending = false;
            mCaptureRequested = false;
        }
    }

    return CaptureResult::kSuccess;
}

void CwbClient::captureWorkerThread() {
    pthread_setname_np(pthread_self(), "UDalsCwbWorker");

    while (true) {
        uint64_t currentGen = 0;
        bool inLockout = false;
        {
            std::unique_lock<std::mutex> lock(mControlMutex);
            mCv.wait(lock, [this] {
                return !mRunning || (!mPaused && !mInvalidRoi.load() && hasActiveBacklight() &&
                                     !mAboveBleedRelevantLux && (mCaptureRequested || mInLockout));
            });
            if (!mRunning) {
                break;
            }
            currentGen = mCaptureGen;
            inLockout = mInLockout;
        }

        if (currentGen != mLastSeenGen) {
            mInitFailures = 0;
            mConsecutiveErrors = 0;
            mPendingStuckCount = 0;
            mHasDispatchedCapture = false;
            mRetryAfter = {};
            mLastLockoutProbeTime = {};
            mLastSeenGen = currentGen;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration waitDuration{};
        if (inLockout) {
            if (mLastLockoutProbeTime.time_since_epoch().count() != 0) {
                auto elapsed = now - mLastLockoutProbeTime;
                if (elapsed < kLockoutProbeInterval) {
                    waitDuration = kLockoutProbeInterval - elapsed;
                }
            }
        } else {
            bool isPending = (mCallbackHandler != nullptr && mCallbackHandler->isPending());
            if (!isPending && mHasDispatchedCapture) {
                auto elapsed = now - mLastDispatchTime;
                if (elapsed < kMinDispatchInterval) {
                    waitDuration = std::max(waitDuration, kMinDispatchInterval - elapsed);
                }
            }
            if (mRetryAfter.time_since_epoch().count() != 0 && now < mRetryAfter) {
                waitDuration = std::max(waitDuration, mRetryAfter - now);
            }
        }

        if (waitDuration > std::chrono::steady_clock::duration::zero()) {
            std::unique_lock<std::mutex> lock(mControlMutex);
            if (mRunning && !mPaused && mCaptureGen == currentGen && mInLockout == inLockout &&
                !mAboveBleedRelevantLux && (inLockout || mCaptureRequested) &&
                hasActiveBacklight()) {
                mCv.wait_for(lock, waitDuration, [&] {
                    return !mRunning || mPaused || mCaptureGen != currentGen ||
                           mInLockout != inLockout || mAboveBleedRelevantLux ||
                           (!inLockout && !mCaptureRequested) || !hasActiveBacklight();
                });
            }
            if (!mRunning) {
                break;
            }
            continue;
        }

        auto attemptStart = std::chrono::steady_clock::now();
        CaptureResult result = requestCwbSample(currentGen);

        if (mInvalidRoi.load() || result == CaptureResult::kSuccess ||
            result == CaptureResult::kAborted) {
            continue;
        }

        ++mConsecutiveErrors;
        bool enterLockout = (mConsecutiveErrors >= kLockoutThreshold);
        if (mCallbackHandler != nullptr && mCallbackHandler->isPending()) {
            ++mPendingStuckCount;
        } else {
            mPendingStuckCount = 0;
        }
        bool clearNow = false;
        {
            std::lock_guard<std::mutex> lock(mControlMutex);
            if (!mRunning) {
                break;
            }
            if (!mPaused && mCaptureGen == currentGen) {
                if (enterLockout) {
                    if (!mInLockout) {
                        clearNow = true;
                        mInLockout = true;
                    }
                    mUpwardCapturePending = false;
                    mCaptureRequested = false;
                } else if (!mInLockout) {
                    mCaptureRequested = true;
                }
            }
        }

        if (clearNow && mCompensator != nullptr) {
            mCompensator->clearSample();
        }

        if (enterLockout && mPendingStuckCount >= kLockoutThreshold) {
            // A pending callback never arrived (lost HWC request or composer restart mid-flight):
            // tear down the stuck session and buffer so lockout probes can reconnect cleanly.
            releaseCwbWorker(/*freeBuffer=*/true);
            mPendingStuckCount = 0;
        }

        if (enterLockout) {
            mLastLockoutProbeTime = std::chrono::steady_clock::now();
            mRetryAfter = {};
        } else {
            mRetryAfter = attemptStart + kMinDispatchInterval;
        }
    }
}
