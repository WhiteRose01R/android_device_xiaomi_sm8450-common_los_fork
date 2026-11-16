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

namespace aidl::vendor::qti::hardware::display::config {
class IDisplayConfig;
}
class CwbCallbackHandler;

class CwbClient {
  public:
    CwbClient(UdalsGeometry geo, std::shared_ptr<UDalsBleedCompensator> compensator);
    ~CwbClient();

    bool start();
    void stop();
    void notifyBacklightChanged();

  private:
    void captureWorkerThread();
    bool hasActiveBacklight() const;
    std::shared_ptr<CwbCallbackHandler> getCallbackHandler() const;
    void retireCallbackHandlerLocked(std::shared_ptr<CwbCallbackHandler> handler);
    bool ensureCwbInitializedLocked();
    void releaseCwbLocked(bool freeBuffer);
    bool requestCwbSample(bool* outRefreshTriggered);
    void computeWeightedRgbLocked(const uint8_t* pixels, float* outR, float* outG, float* outB,
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
    std::atomic<bool> mRunning;
    std::atomic<bool> mPaused;
    std::thread mWorker;

    std::mutex mStateMutex;
    std::mutex mLock;
    std::mutex mCaptureLock;
    mutable std::mutex mCallbackLock;
    std::condition_variable mCv;
    uint64_t mCaptureGen = 0;

    std::shared_ptr<aidl::vendor::qti::hardware::display::config::IDisplayConfig> mCwbClient;
    std::shared_ptr<CwbCallbackHandler> mCallbackHandler;
    std::deque<std::shared_ptr<CwbCallbackHandler>> mRetiredCallbacks;
    AHardwareBuffer* mCwbBuffer = nullptr;
    const struct native_handle* mCwbBufferHandle = nullptr;
    uint32_t mBufferStride = 0;
    bool mDisplayBoundsChecked = false;
    std::atomic<bool> mInvalidRoi{false};
    std::atomic<uint32_t> mInitFailures{0};
    std::atomic<uint32_t> mPendingTimeouts{0};
    std::atomic<uint32_t> mConsecutiveErrors{0};
    std::atomic<uint32_t> mRefreshAttempts{0};
    std::chrono::steady_clock::time_point mLastInitAttempt{};
};
