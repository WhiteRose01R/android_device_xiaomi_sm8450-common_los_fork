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

struct UdalsRoi {
    int32_t left = 0;
    int32_t top = 0;
    int32_t right = 0;
    int32_t bottom = 0;
};

class CwbClient {
  public:
    CwbClient(UdalsRoi roi, std::shared_ptr<UDalsBleedCompensator> compensator);
    ~CwbClient();

    bool start();
    void stop();

  private:
    void captureWorkerThread();
    bool ensureCwbInitialized();
    void releaseCwb();
    bool requestCwbSample();

    UdalsRoi mRoi;
    int32_t mRoiWidth;
    int32_t mRoiHeight;
    int32_t mCenterRelX;
    int32_t mCenterRelY;
    int32_t mRadius1;
    int32_t mRadius2;
    int32_t mRadius3;
    int32_t mRadius1Sq;
    int32_t mRadius2Sq;
    int32_t mRadius3Sq;

    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::atomic<bool> mRunning;
    std::thread mWorker;

    std::mutex mLock;
    std::condition_variable mCv;

    void* mDpyConfigLib = nullptr;
    DisplayConfig::ClientInterface* mCwbClient = nullptr;
    std::unique_ptr<CwbCallbackHandler> mCallbackHandler;
    void (*mDestroyFn)(DisplayConfig::ClientInterface*) = nullptr;
    AHardwareBuffer* mCwbBuffer = nullptr;
    const struct native_handle* mCwbBufferHandle = nullptr;
    uint32_t mBufferStride = 0;
    uint32_t mInitFailures = 0;
    uint32_t mPendingTimeouts = 0;
    uint32_t mRefreshAttempts = 0;
    std::chrono::steady_clock::time_point mLastInitAttempt{};
};
