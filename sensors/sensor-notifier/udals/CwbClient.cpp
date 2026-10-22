/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsCwbClient"

#include "CwbClient.h"

#include <android-base/logging.h>
#include <android/hardware_buffer.h>
#include <dlfcn.h>
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

}  // namespace

namespace DisplayConfig {

enum class DisplayType {
    kInvalid = -1,
    kPrimary = 0,
    kExternal = 1,
    kVirtual = 2,
    kBuiltIn2 = 3,
};

enum class ExternalStatus {
    kOffline = 0,
    kOnline = 1,
    kPause = 2,
    kResume = 3,
};

enum class DynRefreshRateOp {
    kDisableMetadata = 0,
    kEnableMetadata = 1,
    kSetFps = 2,
};

struct Rect {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
};

class ConfigCallback {
  public:
    virtual void NotifyCWBBufferDone(int32_t error, const native_handle_t* buffer) = 0;
    virtual void NotifyQsyncChange(bool, int32_t, int32_t) {}
    virtual void NotifyIdleStatus(bool) {}
    virtual ~ConfigCallback() = default;
};

class ClientInterface {
  public:
    virtual int IsDisplayConnected(DisplayType type, bool* connected) = 0;
    virtual int SetDisplayStatus(DisplayType type, ExternalStatus status) = 0;
    virtual int ConfigureDynRefreshRate(DynRefreshRateOp op, uint32_t hz) = 0;
    virtual int GetConfigCount(DisplayType type, uint32_t* count) = 0;
    virtual int GetActiveConfig(DisplayType type, uint32_t* index) = 0;
    virtual int SetActiveConfig(DisplayType type, uint32_t index) = 0;
    virtual int GetDisplayAttributes(uint32_t index, DisplayType type, void* attr) = 0;
    virtual int SetPanelBrightness(uint32_t level) = 0;
    virtual int GetPanelBrightness(uint32_t* level) = 0;
    virtual int MinHdcpEncryptionLevelChanged(DisplayType type, uint32_t min_enc_level) = 0;
    virtual int RefreshScreen() = 0;
    virtual int ControlPartialUpdate(DisplayType type, bool enable) = 0;
    virtual int ToggleScreenUpdate(bool on) = 0;
    virtual int SetIdleTimeout(uint32_t value) = 0;
    virtual int GetHDRCapabilities(DisplayType type, void* caps) = 0;
    virtual int SetCameraLaunchStatus(uint32_t on) = 0;
    virtual ~ClientInterface() = default;
    virtual int DisplayBWTransactionPending(bool* status) = 0;
    virtual int SetDisplayAnimating(uint64_t display_id, bool animating) = 0;
    virtual int ControlIdlePowerCollapse(bool enable, bool synchronous) = 0;
    virtual int GetWriteBackCapabilities(bool* status) = 0;
    virtual int SetDisplayDppsAdROI(uint32_t display_id, uint32_t h_start, uint32_t h_end,
                                    uint32_t v_start, uint32_t v_end, uint32_t factor_in,
                                    uint32_t factor_out) = 0;
    virtual int UpdateVSyncSourceOnPowerModeOff() = 0;
    virtual int UpdateVSyncSourceOnPowerModeDoze() = 0;
    virtual int SetPowerMode(uint32_t display_id, uint32_t power_mode) = 0;
    virtual int IsPowerModeOverrideSupported(uint32_t display_id, bool* supported) = 0;
    virtual int IsHDRSupported(uint32_t display_id, bool* supported) = 0;
    virtual int IsWCGSupported(uint32_t display_id, bool* supported) = 0;
    virtual int SetLayerAsMask(uint32_t display_id, uint64_t layer_id) = 0;
    virtual int GetDebugProperty(std::string prop_name, std::string* value) = 0;
    virtual int GetActiveBuiltinDisplayAttributes(void* attr) = 0;
    virtual int SetPanelLuminanceAttributes(uint32_t display_id, float min_lum, float max_lum) = 0;
    virtual int IsBuiltInDisplay(uint32_t display_id, bool* is_builtin) = 0;
    virtual int IsAsyncVDSCreationSupported(bool* supported) = 0;
    virtual int CreateVirtualDisplay(uint32_t width, uint32_t height, int format) = 0;
    virtual int GetSupportedDSIBitClks(uint32_t display_id, std::vector<uint64_t>* bit_clks) = 0;
    virtual int GetDSIClk(uint32_t display_id, uint64_t* bit_clk) = 0;
    virtual int SetDSIClk(uint32_t display_id, uint64_t bit_clk) = 0;
    virtual int SetCWBOutputBuffer(uint32_t display_id, Rect rect, bool dither,
                                   const native_handle_t* buffer) = 0;
};

}  // namespace DisplayConfig

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
        mActiveRequestId = 0;
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
      mCallbackHandler(std::make_unique<CwbCallbackHandler>()) {
    int32_t maxRadius = std::min(mRoiWidth, mRoiHeight) / 2;
    mRadius1 = static_cast<int32_t>(std::lround(maxRadius * 0.38f));
    mRadius2 = static_cast<int32_t>(std::lround(maxRadius * 0.72f));
    mRadius3 = maxRadius;
    mRadius1Sq = mRadius1 * mRadius1;
    mRadius2Sq = mRadius2 * mRadius2;
    mRadius3Sq = mRadius3 * mRadius3;
}

CwbClient::~CwbClient() {
    stop();
    releaseCwb();
}

bool CwbClient::ensureCwbInitialized() {
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

    // Loaded via dlopen as libdisplayconfig.qti is a proprietary vendor library
    mDpyConfigLib = dlopen("libdisplayconfig.qti.so", RTLD_NOW);
    if (mDpyConfigLib == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "Failed to dlopen libdisplayconfig.qti.so: " << dlerror() << " (attempt "
                     << mInitFailures << ")";
        releaseCwb();
        return false;
    }

    using CreateFn =
            int (*)(std::string, DisplayConfig::ConfigCallback*, DisplayConfig::ClientInterface**);
    using DestroyFn = void (*)(DisplayConfig::ClientInterface*);

    auto createFn = reinterpret_cast<CreateFn>(
            dlsym(mDpyConfigLib,
                  "_ZN13DisplayConfig15ClientInterface6CreateENSt3__112basic_stringIcNS1_11char_"
                  "traitsIcEENS1_9allocatorIcEEEEPNS_14ConfigCallbackEPPS0_"));
    auto destroyFn = reinterpret_cast<DestroyFn>(
            dlsym(mDpyConfigLib, "_ZN13DisplayConfig15ClientInterface7DestroyEPS0_"));

    if (createFn == nullptr || destroyFn == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "Failed to find DisplayConfig ClientInterface symbols"
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }
    mDestroyFn = destroyFn;

    DisplayConfig::ClientInterface* client = nullptr;
    if (createFn("sensor-notifier", mCallbackHandler.get(), &client) != 0 || client == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "DisplayConfig ClientInterface Create failed"
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }
    mCwbClient = client;

    AHardwareBuffer_Desc desc = {
            .width = static_cast<uint32_t>(mRoiWidth),
            .height = static_cast<uint32_t>(mRoiHeight),
            .layers = 1,
            .format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM,
            .usage = AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN | AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT,
            .stride = 0,
            .rfu0 = 0,
            .rfu1 = 0,
    };

    if (AHardwareBuffer_allocate(&desc, &mCwbBuffer) != 0 || mCwbBuffer == nullptr) {
        mInitFailures++;
        LOG(WARNING) << "Failed to allocate CWB buffer"
                     << " (attempt " << mInitFailures << ")";
        releaseCwb();
        return false;
    }

    AHardwareBuffer_describe(mCwbBuffer, &desc);
    mCwbBufferHandle = AHardwareBuffer_getNativeHandle(mCwbBuffer);
    mBufferStride = desc.stride;
    mInitFailures = 0;
    LOG(INFO) << "CwbClient initialized hardware CWB: " << mRoiWidth << "x" << mRoiHeight
              << " RGBA, stride=" << mBufferStride;
    return true;
}

void CwbClient::releaseCwb() {
    if (mCwbClient != nullptr && mDestroyFn != nullptr) {
        mDestroyFn(mCwbClient);
        mCwbClient = nullptr;
    }

    if (mCwbBuffer != nullptr) {
        AHardwareBuffer_release(mCwbBuffer);
        mCwbBuffer = nullptr;
    }
    mCwbBufferHandle = nullptr;

    // Keep libdisplayconfig.qti.so mapped in the process address space during
    // daemon lifetime so any late in-flight return paths do not execute against unmapped code.
    mDestroyFn = nullptr;
}

bool CwbClient::start() {
    std::lock_guard<std::mutex> lock(mLock);
    if (mRunning) {
        return true;
    }

    if (mCallbackHandler != nullptr) {
        mCallbackHandler->forceReset();
    }
    mInitFailures = 0;
    mPendingTimeouts = 0;
    mRefreshAttempts = 0;
    mRunning = true;
    mWorker = std::thread(&CwbClient::captureWorkerThread, this);
    return true;
}

void CwbClient::stop() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        if (!mRunning) {
            return;
        }
        mRunning = false;
    }

    if (mCallbackHandler != nullptr) {
        mCallbackHandler->abort();
    }

    mCv.notify_all();

    if (mWorker.joinable()) {
        mWorker.join();
    }

    if (mCallbackHandler != nullptr) {
        if (mCallbackHandler->isPending()) {
            mCallbackHandler->waitForDrain(std::chrono::milliseconds(80));
        }
        mCallbackHandler->forceReset();
    }
    mPendingTimeouts = 0;
    mRefreshAttempts = 0;
}

bool CwbClient::requestCwbSample() {
    if (!mRunning || mCompensator == nullptr || mCompensator->getBacklight() == 0) {
        return false;
    }

    if (!ensureCwbInitialized()) {
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
                if (mRunning && !mCompensator->hasSample() && mRefreshAttempts < 3) {
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

        int ret =
                mCwbClient->SetCWBOutputBuffer(kPrimaryDisplayId, roiRect, true, mCwbBufferHandle);
        if (ret != 0) {
            mCallbackHandler->cancelPending();
            return false;
        }

        if (mRunning && !mCompensator->hasSample() && mRefreshAttempts < 3) {
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

    void* vaddr = nullptr;
    if (AHardwareBuffer_lock(mCwbBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN, -1, nullptr,
                             &vaddr) != 0 ||
        vaddr == nullptr) {
        return false;
    }

    const uint8_t* pixels = static_cast<const uint8_t*>(vaddr);
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

    AHardwareBuffer_unlock(mCwbBuffer, nullptr);

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
        bool sampled = requestCwbSample();

        auto sleepDuration = (mInitFailures >= 15) ? std::chrono::seconds(5) : kSampleInterval;
        if (mCompensator != nullptr && mCompensator->getBacklight() == 0) {
            sleepDuration = std::chrono::milliseconds(200);
        } else if (!sampled && mPendingTimeouts > 0) {
            sleepDuration = std::chrono::milliseconds(150);
        }

        std::unique_lock<std::mutex> lock(mLock);
        mCv.wait_for(lock, sleepDuration, [this] { return !mRunning; });
    }
}
