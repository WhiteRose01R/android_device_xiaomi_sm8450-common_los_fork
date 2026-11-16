/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsCwbClient"

#include "CwbClient.h"

#include <aidl/vendor/qti/hardware/display/config/BnDisplayConfigCallback.h>
#include <aidl/vendor/qti/hardware/display/config/IDisplayConfig.h>
#include <aidlcommonsupport/NativeHandle.h>
#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <android/hardware_buffer.h>
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

using aidl::android::hardware::common::NativeHandle;
using aidl::vendor::qti::hardware::display::config::Attributes;
using aidl::vendor::qti::hardware::display::config::BnDisplayConfigCallback;
using aidl::vendor::qti::hardware::display::config::CameraSmoothOp;
using aidl::vendor::qti::hardware::display::config::Concurrency;
using aidl::vendor::qti::hardware::display::config::DisplayType;
using aidl::vendor::qti::hardware::display::config::IDisplayConfig;
using aidl::vendor::qti::hardware::display::config::Rect;
using aidl::vendor::qti::hardware::display::config::TUIEventType;

namespace {

constexpr int32_t kPrimaryDisplayId = static_cast<int32_t>(DisplayType::PRIMARY);

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
            mLocked = (err == 0);
            if (mLocked) {
                CHECK(mAddr != nullptr) << "AHardwareBuffer_lock succeeded with null address";
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

class CwbCallbackHandler : public BnDisplayConfigCallback {
  public:
    ndk::ScopedAStatus notifyCWBBufferDone(int32_t error,
                                           const NativeHandle& /* buffer */) override {
        std::lock_guard<std::mutex> lock(mMutex);
        if (!mPending) {
            return ndk::ScopedAStatus::ok();
        }
        mError = error;
        mDone = true;
        mPending = false;
        mCv.notify_all();
        return ndk::ScopedAStatus::ok();
    }

    ndk::ScopedAStatus notifyQsyncChange(bool, int32_t, int32_t) override {
        return ndk::ScopedAStatus::ok();
    }

    ndk::ScopedAStatus notifyIdleStatus(bool) override { return ndk::ScopedAStatus::ok(); }

    ndk::ScopedAStatus notifyCameraSmoothInfo(CameraSmoothOp, int32_t) override {
        return ndk::ScopedAStatus::ok();
    }

    ndk::ScopedAStatus notifyResolutionChange(int32_t, const Attributes&) override {
        return ndk::ScopedAStatus::ok();
    }

    ndk::ScopedAStatus notifyFpsMitigation(int32_t, const Attributes&, Concurrency) override {
        return ndk::ScopedAStatus::ok();
    }

    ndk::ScopedAStatus notifyTUIEventDone(int32_t, DisplayType, TUIEventType) override {
        return ndk::ScopedAStatus::ok();
    }

    void prepareForCapture() {
        std::lock_guard<std::mutex> lock(mMutex);
        mDone = false;
        mAborted = false;
        mError = 0;
        mPending = true;
    }

    void cancelPending() {
        std::lock_guard<std::mutex> lock(mMutex);
        mPending = false;
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

    void forceReset(bool clearAborted = true) {
        std::lock_guard<std::mutex> lock(mMutex);
        mDone = false;
        mPending = false;
        mError = 0;
        if (clearAborted) {
            mAborted = false;
        }
    }

  private:
    mutable std::mutex mMutex;
    std::condition_variable mCv;
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
      mCompensator(std::move(compensator)),
      mRunning(false),
      mPaused(true) {
    mRadius1Sq = geo.r1 * geo.r1;
    mRadius2Sq = geo.r2 * geo.r2;
    mRadius3Sq = geo.r3 * geo.r3;
}

CwbClient::~CwbClient() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mRunning = false;
        mPaused = true;
    }

    std::shared_ptr<CwbCallbackHandler> cb = getCallbackHandler();
    if (cb != nullptr) {
        cb->abort();
    }

    mCv.notify_all();

    if (mWorker.joinable()) {
        mWorker.join();
    }

    std::lock_guard<std::mutex> capLock(mCaptureLock);
    releaseCwbLocked(true);
    mRetiredCallbacks.clear();
}

bool CwbClient::hasActiveBacklight() const {
    return mCompensator == nullptr || mCompensator->getBacklight() > 0;
}

void CwbClient::notifyBacklightChanged() {
    // Acquire mLock to serialize with mCv.wait() in captureWorkerThread(),
    // since the backlight state itself is guarded by mCompensator's mutex.
    std::lock_guard<std::mutex> lock(mLock);
    mCv.notify_all();
}

std::shared_ptr<CwbCallbackHandler> CwbClient::getCallbackHandler() const {
    std::lock_guard<std::mutex> cbLock(mCallbackLock);
    return mCallbackHandler;
}

void CwbClient::retireCallbackHandlerLocked(std::shared_ptr<CwbCallbackHandler> handler) {
    if (handler == nullptr) {
        return;
    }
    // Bounded retire queue: guards against a late notifyCWBBufferDone firing
    // after the proxy is dropped (AIDL has no explicit Destroy; binder death
    // races with in-flight calls). Assumes >8 simultaneous in-flight callbacks
    // is impossible; each drains within ~80 ms in practice.
    constexpr size_t kMaxRetiredCallbacks = 8;
    mRetiredCallbacks.push_back(std::move(handler));
    while (mRetiredCallbacks.size() > kMaxRetiredCallbacks) {
        mRetiredCallbacks.pop_front();
    }
}

bool CwbClient::ensureCwbInitializedLocked() {
    if (mInvalidRoi.load() || mGeo.left < 0 || mGeo.top < 0 || mRoiWidth <= 0 || mRoiHeight <= 0) {
        return false;
    }

    if (mCwbClient != nullptr && mCwbBufferHandle != nullptr) {
        return true;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t failures = mInitFailures.load();
    auto minInterval = (failures >= 15) ? std::chrono::seconds(5) : std::chrono::seconds(1);
    if (failures > 0 && std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - mLastInitAttempt) < minInterval) {
        return false;
    }
    mLastInitAttempt = now;

    if (mCwbClient == nullptr) {
        static std::once_flag sBinderThreadPoolOnce;
        std::call_once(sBinderThreadPoolOnce, []() { ABinderProcess_startThreadPool(); });

        const std::string instance = std::string(IDisplayConfig::descriptor) + "/default";
        ndk::SpAIBinder binder(AServiceManager_checkService(instance.c_str()));
        std::shared_ptr<IDisplayConfig> client = IDisplayConfig::fromBinder(binder);
        if (client == nullptr) {
            uint32_t attempt = ++mInitFailures;
            LOG(WARNING) << "Failed to connect to " << instance << " (attempt " << attempt << ")";
            releaseCwbLocked(false);
            return false;
        }
        auto callbackHandler = ndk::SharedRefBase::make<CwbCallbackHandler>();
        {
            std::lock_guard<std::mutex> cbLock(mCallbackLock);
            mCallbackHandler = std::move(callbackHandler);
        }
        mCwbClient = std::move(client);
    }

    if (!mDisplayBoundsChecked) {
        Attributes attrs{};
        if (!mCwbClient->getActiveBuiltinDisplayAttributes(&attrs).isOk() || attrs.xRes <= 0 ||
            attrs.yRes <= 0) {
            uint32_t attempt = ++mInitFailures;
            LOG(WARNING) << "Failed to query active builtin display attributes (attempt " << attempt
                         << ")";
            releaseCwbLocked(true);
            return false;
        }
        mDisplayBoundsChecked = true;
        if (mGeo.right > attrs.xRes || mGeo.bottom > attrs.yRes) {
            LOG(ERROR) << "UDALS CWB ROI (" << mGeo.left << "," << mGeo.top << "," << mGeo.right
                       << "," << mGeo.bottom << ") exceeds active display resolution " << attrs.xRes
                       << "x" << attrs.yRes << "; disabling CWB";
            mInvalidRoi = true;
            releaseCwbLocked(true);
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
            releaseCwbLocked(true);
            return false;
        }

        AHardwareBuffer_describe(mCwbBuffer, &desc);
        mBufferStride = desc.stride;
        if (mBufferStride < static_cast<uint32_t>(mRoiWidth) ||
            desc.height < static_cast<uint32_t>(mRoiHeight)) {
            mInitFailures++;
            LOG(ERROR) << "Allocated CWB buffer smaller than ROI";
            releaseCwbLocked(true);
            return false;
        }

        mCwbBufferHandle = AHardwareBuffer_getNativeHandle(mCwbBuffer);
        if (mCwbBufferHandle == nullptr) {
            mInitFailures++;
            LOG(WARNING) << "Failed to get native handle for CWB buffer";
            releaseCwbLocked(true);
            return false;
        }

        LOG(INFO) << "CWB client initialized successfully: " << mRoiWidth << "x" << mRoiHeight
                  << " stride=" << mBufferStride;
    }

    return true;
}

void CwbClient::releaseCwbLocked(bool freeBuffer) {
    std::shared_ptr<CwbCallbackHandler> retired;
    {
        std::lock_guard<std::mutex> cbLock(mCallbackLock);
        retired = std::move(mCallbackHandler);
    }
    if (retired != nullptr) {
        retired->abort();
        retired->cancelPending();
        retireCallbackHandlerLocked(std::move(retired));
    }

    if (freeBuffer) {
        if (mCwbBuffer != nullptr) {
            AHardwareBuffer_release(mCwbBuffer);
            mCwbBuffer = nullptr;
            mCwbBufferHandle = nullptr;
        }
        mBufferStride = 0;
    }

    mCwbClient.reset();
}

bool CwbClient::start() {
    if (mInvalidRoi.load()) {
        return false;
    }
    std::lock_guard<std::mutex> stateLock(mStateMutex);
    {
        std::lock_guard<std::mutex> capLock(mCaptureLock);
        mCaptureGen++;
        mInitFailures = 0;
        mPendingTimeouts = 0;
        mConsecutiveErrors = 0;
        mRefreshAttempts = 0;
        std::shared_ptr<CwbCallbackHandler> cb = getCallbackHandler();
        if (cb != nullptr && !cb->isPending()) {
            cb->forceReset();
        }
    }
    {
        std::lock_guard<std::mutex> lock(mLock);
        mPaused = false;
        if (!mRunning) {
            mRunning = true;
            mWorker = std::thread(&CwbClient::captureWorkerThread, this);
        }
    }
    mCv.notify_all();
    return true;
}

void CwbClient::stop() {
    std::lock_guard<std::mutex> stateLock(mStateMutex);
    {
        std::lock_guard<std::mutex> lock(mLock);
        if (!mRunning || mPaused) {
            return;
        }
        mPaused = true;
    }

    std::shared_ptr<CwbCallbackHandler> cb = getCallbackHandler();
    if (cb != nullptr) {
        cb->abort();
    }

    mCv.notify_all();

    std::lock_guard<std::mutex> capLock(mCaptureLock);
    mCaptureGen++;
    cb = getCallbackHandler();
    if (cb != nullptr) {
        if (cb->isPending() && !cb->waitForDrain(std::chrono::milliseconds(80))) {
            // Drop the session while keeping the buffer if a pre-sleep capture failed to drain.
            releaseCwbLocked(/*freeBuffer=*/false);
        } else {
            // Keep mAborted set until next start() so any concurrent waitForDone() exits
            // immediately.
            cb->forceReset(/*clearAborted=*/false);
        }
    }
    mPendingTimeouts = 0;
    mConsecutiveErrors = 0;
    mRefreshAttempts = 0;
}

void CwbClient::computeWeightedRgbLocked(const uint8_t* pixels, float* outR, float* outG,
                                         float* outB, float* outTotalWeight) const {
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

    uint64_t captureGen = 0;
    int32_t cwbError = 0;
    bool bufferReady = false;
    bool wasPending = false;
    std::shared_ptr<CwbCallbackHandler> cb;

    {
        std::lock_guard<std::mutex> capLock(mCaptureLock);
        if (!mRunning || mPaused || mCompensator == nullptr) {
            return false;
        }

        if (mCompensator->getBacklight() == 0) {
            return false;
        }

        if (!ensureCwbInitializedLocked() || mCwbBufferHandle == nullptr) {
            return false;
        }

        cb = getCallbackHandler();
        if (cb == nullptr) {
            return false;
        }

        captureGen = mCaptureGen;
        bufferReady = cb->consumeIfDone(&cwbError);

        if (!bufferReady) {
            wasPending = cb->isPending();
            if (!wasPending) {
                Rect roiRect = {
                        .left = mGeo.left,
                        .top = mGeo.top,
                        .right = mGeo.right,
                        .bottom = mGeo.bottom,
                };

                cb->prepareForCapture();
                ndk::ScopedAStatus status =
                        mCwbClient->setCWBOutputBuffer(cb, kPrimaryDisplayId, roiRect, true,
                                                       ::android::dupToAidl(mCwbBufferHandle));
                if (!status.isOk()) {
                    cb->cancelPending();
                    if (status.getStatus() == STATUS_DEAD_OBJECT) {
                        releaseCwbLocked(/*freeBuffer=*/false);
                    }
                    ++mConsecutiveErrors;
                    return false;
                }
            }
        }
    }

    if (!bufferReady) {
        auto waitMs = wasPending ? std::chrono::milliseconds(20) : std::chrono::milliseconds(80);
        if (cb->waitForDone(waitMs, &cwbError)) {
            bufferReady = true;
        } else {
            std::lock_guard<std::mutex> capLock(mCaptureLock);
            if (!mRunning || mPaused || captureGen != mCaptureGen || mCompensator == nullptr) {
                return false;
            }
            uint32_t timeouts = ++mPendingTimeouts;
            if (!mCompensator->hasSample() && mRefreshAttempts < 3 && mCwbClient != nullptr) {
                mCwbClient->refreshScreen();
                mRefreshAttempts++;
                if (outRefreshTriggered != nullptr) {
                    *outRefreshTriggered = true;
                }
            } else if (timeouts >= (mCompensator->hasSample() ? 10u : 5u)) {
                releaseCwbLocked(/*freeBuffer=*/true);
                mInitFailures++;
                mPendingTimeouts = 0;
                mRefreshAttempts = 0;
            }
            return false;
        }
    }

    std::lock_guard<std::mutex> capLock(mCaptureLock);
    if (!mRunning || mPaused || captureGen != mCaptureGen || mCompensator == nullptr ||
        mCwbBuffer == nullptr) {
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
    computeWeightedRgbLocked(static_cast<const uint8_t*>(scopedLock.address()), &weightedR,
                             &weightedG, &weightedB, &totalWeight);

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

    while (mRunning) {
        {
            std::unique_lock<std::mutex> lock(mLock);
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

        if (mRunning && !mPaused && !hasActiveBacklight()) {
            continue;
        }

        auto sleepDuration = kSampleInterval;
        uint32_t initFailures = mInitFailures.load();
        uint32_t pendingTimeouts = mPendingTimeouts.load();
        uint32_t consecutiveErrors = mConsecutiveErrors.load();
        if (initFailures >= 15 || consecutiveErrors >= 15) {
            sleepDuration = std::chrono::seconds(5);
        } else if (refreshTriggered) {
            // Re-check within ~2 vsyncs so initial wake compensation beats the wake flare gate.
            sleepDuration = std::chrono::milliseconds(30);
        } else if (!sampled && (pendingTimeouts > 0 || consecutiveErrors > 0 || initFailures > 0)) {
            sleepDuration = (pendingTimeouts >= 5 || consecutiveErrors >= 5 || initFailures >= 5)
                                    ? std::chrono::milliseconds(500)
                                    : std::chrono::milliseconds(300);
        }

        std::unique_lock<std::mutex> lock(mLock);
        mCv.wait_for(lock, sleepDuration, [this] { return !mRunning || mPaused; });
    }
}
