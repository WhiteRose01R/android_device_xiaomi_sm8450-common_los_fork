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
    void setSampledRgb(float r, float g, float b);
    bool compensate(float rawLux, float* outLux);
    bool hasSample() const;
    uint32_t getBacklight() const;
    uint32_t getNormalMaxBacklight() const;

  private:
    float calculateBleedLuxLocked() const;
    static float interpolateTable(const std::vector<CaliPoint>& table, float brightness);

    mutable std::mutex mLock;
    uint32_t mBacklight;
    uint32_t mNormalMaxBacklight;
    bool mHasSample;
    bool mWakeGateActive;
    float mNormR;
    float mNormG;
    float mNormB;
    float mPeakBleedLuxRed;
    float mPeakBleedLuxGreen;
    float mPeakBleedLuxBlue;
    std::chrono::steady_clock::time_point mResetTime{};

    std::vector<CaliPoint> mNormalDataR;
    std::vector<CaliPoint> mNormalDataG;
    std::vector<CaliPoint> mNormalDataB;

    std::array<float, 5> mMedianWindow;
    size_t mMedianCount;
    size_t mMedianIndex;
};
