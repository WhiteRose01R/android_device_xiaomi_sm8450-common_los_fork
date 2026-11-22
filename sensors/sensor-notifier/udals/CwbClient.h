/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "UDalsBleedCompensator.h"

struct AHardwareBuffer;
struct native_handle;

namespace DisplayConfig {
class ClientInterface;
}
class CwbCallbackHandler;

class CwbClient {
  public:
    CwbClient(UdalsGeometry geo, std::shared_ptr<UDalsBleedCompensator> compensator);
    ~CwbClient();

    bool start();
    void stop();
    void notifyBacklightChanged();
    void onRawLuxSample(float rawLux);

  private:
    enum class CaptureResult {
        kSuccess,
        kFailed,
        kAborted,
    };

    void captureWorkerThread();
    bool hasActiveBacklight() const;
    bool isActiveGen(uint64_t gen) const;
    void retireCallbackHandlerWorker(std::shared_ptr<CwbCallbackHandler> handler);
    bool ensureCwbInitializedWorker();
    void releaseCwbWorker(bool freeBuffer);
    void triggerRefreshWorker();
    CaptureResult requestCwbSample(uint64_t currentGen);
    void computeWeightedRgb(const uint8_t* pixels, float* outR, float* outG, float* outB,
                            float* outTotalWeight) const;

    UdalsGeometry mGeo;
    int32_t mRoiWidth;
    int32_t mRoiHeight;
    int32_t mCenterRelX;
    int32_t mCenterRelY;
    int32_t mRadius1Sq;
    int32_t mRadius2Sq;
    int32_t mRadius3Sq;

    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::atomic<bool> mInvalidRoi{false};

    mutable std::mutex mControlMutex;
    std::condition_variable mCv;
    bool mRunning = false;
    bool mPaused = true;
    uint64_t mCaptureGen = 0;
    bool mCaptureRequested = false;
    bool mInLockout = false;
    bool mHasLatestRawLux = false;
    float mLatestRawLux = 0.0f;
    bool mHasRawLuxAnchor = false;
    float mLastTriggerRawLux = 0.0f;
    bool mSeedAnchorOnNextSample = false;
    std::shared_ptr<CwbCallbackHandler> mActiveCallback;
    std::thread mWorker;

    // Worker-thread-exclusive CWB session and buffer state.
    uint64_t mLastSeenGen = 0;
    DisplayConfig::ClientInterface* mCwbClient = nullptr;
    std::shared_ptr<CwbCallbackHandler> mCallbackHandler;
    std::deque<std::shared_ptr<CwbCallbackHandler>> mRetiredCallbacks;
    AHardwareBuffer* mCwbBuffer = nullptr;
    const struct native_handle* mCwbBufferHandle = nullptr;
    uint32_t mBufferStride = 0;
    bool mDisplayBoundsChecked = false;
    uint32_t mInitFailures = 0;
    uint32_t mConsecutiveErrors = 0;
    bool mHasDispatchedCapture = false;
    std::chrono::steady_clock::time_point mLastInitAttempt{};
    std::chrono::steady_clock::time_point mLastDispatchTime{};
    std::chrono::steady_clock::time_point mRetryAfter{};
    std::chrono::steady_clock::time_point mLastLockoutProbeTime{};
};
