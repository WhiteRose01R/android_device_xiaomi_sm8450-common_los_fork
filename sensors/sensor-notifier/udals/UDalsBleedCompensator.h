/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>

class UDalsBleedCompensator {
  public:
    struct CaliPoint {
        float brightness;
        float clear;
    };

    static constexpr size_t kTableSize = 24;

    UDalsBleedCompensator();
    ~UDalsBleedCompensator() = default;

    void reset();
    void setBacklight(uint32_t brightness);
    void setDcMode(bool dcMode);
    void setHbmMode(bool hbm);
    void setSampledRgb(float r, float g, float b);
    bool compensate(float rawLux, float* outLux);
    bool hasSample() const;
    uint32_t getBacklight() const;

  private:
    float calculateBleedLuxLocked() const;

    static float interpolateTable(const std::array<CaliPoint, kTableSize>& table, float brightness);

    mutable std::mutex mLock;
    uint32_t mBacklight;
    bool mDcMode;
    bool mHbmMode;
    bool mHasSample;
    float mNormR;
    float mNormG;
    float mNormB;
    float mPeakBleedLuxRed;
    float mPeakBleedLuxGreen;
    float mPeakBleedLuxBlue;
    std::chrono::steady_clock::time_point mResetTime{};

    std::array<float, 5> mMedianWindow;
    size_t mMedianCount;
    size_t mMedianIndex;
    float mLastCompensatedLux = 0.0f;
};
