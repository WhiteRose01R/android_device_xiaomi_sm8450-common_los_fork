/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

struct UdalsGeometry {
    int32_t cx = 0;
    int32_t cy = 0;
    int32_t left = 0;
    int32_t top = 0;
    int32_t right = 0;
    int32_t bottom = 0;
    int32_t r1 = 0;
    int32_t r2 = 0;
    int32_t r3 = 0;
};

class UDalsBleedCompensator {
  public:
    struct CaliPoint {
        float brightness;
        float clear;
    };

    UDalsBleedCompensator();
    ~UDalsBleedCompensator() = default;

    bool loadConfig(const std::string& path, UdalsGeometry* outGeo);

    void reset();
    void clearSample();
    void setBacklight(uint32_t brightness);
    void setDcMode(bool dcMode);
    void setSampledRgb(float r, float g, float b);
    void setSampledRgb(float rHigh, float gHigh, float bHigh, float rLow, float gLow, float bLow);
    void updateTransitionHold(bool upwardPending, float rawLux, uint64_t expectedSeq);
    void clearTransitionHold();
    bool compensate(float rawLux, float* outLux);
    bool hasSample() const;
    uint64_t getSampleSeq() const;
    uint32_t getBacklight() const;
    uint32_t getNormalMaxBacklight() const;

  private:
    struct BacklightHoldEntry {
        std::chrono::steady_clock::time_point endTime;
        uint32_t brightness;
    };

    void pruneBacklightHistoryLocked(std::chrono::steady_clock::time_point now);
    uint32_t effectiveBacklightLocked(std::chrono::steady_clock::time_point now);
    float calculateBleedLuxForRgbLocked(uint32_t backlight, float r, float g, float b) const;
    float calculateBleedLuxLocked(uint32_t effectiveBacklight) const;
    float calculateMaxWhiteBleedLocked(uint32_t effectiveBacklight) const;
    void resetResidualFilterLocked();
    float filterSignedResidualLocked(float signedRes, float bleed);
    static float interpolateTable(const std::vector<CaliPoint>& table, float brightness);

    mutable std::mutex mLock;
    uint32_t mBacklight;
    uint32_t mNormalMaxBacklight;
    bool mDcMode;
    bool mHasSample;
    bool mWakeGateActive;
    bool mTransitionHoldActive;
    bool mTransitionHoldTimedOut;
    bool mHasSettledOutput;
    bool mHasResidualEma;
    uint64_t mSampleSeq;
    float mNormR;
    float mNormG;
    float mNormB;
    float mNormLowR;
    float mNormLowG;
    float mNormLowB;
    float mLastSettledCompLux;
    float mResidualEma;
    float mPeakBleedLuxRed;
    float mPeakBleedLuxGreen;
    float mPeakBleedLuxBlue;
    std::chrono::steady_clock::time_point mResetTime{};
    std::chrono::steady_clock::time_point mTransitionHoldStart{};
    std::deque<BacklightHoldEntry> mBacklightHistory;
    std::deque<float> mResidualWindow;

    std::vector<CaliPoint> mNormalDataR;
    std::vector<CaliPoint> mNormalDataG;
    std::vector<CaliPoint> mNormalDataB;
    std::vector<CaliPoint> mDcDataR;
    std::vector<CaliPoint> mDcDataG;
    std::vector<CaliPoint> mDcDataB;
};
