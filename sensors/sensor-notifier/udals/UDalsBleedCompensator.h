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
#include <optional>
#include <string>

#include "UDalsStockModel.h"

class UDalsBleedCompensator {
  public:
    UDalsBleedCompensator() = default;
    ~UDalsBleedCompensator() = default;
    UDalsBleedCompensator(const UDalsBleedCompensator&) = delete;
    UDalsBleedCompensator& operator=(const UDalsBleedCompensator&) = delete;
    UDalsBleedCompensator(UDalsBleedCompensator&&) = delete;
    UDalsBleedCompensator& operator=(UDalsBleedCompensator&&) = delete;

    bool loadConfig(UdalsGeometry* outGeo);
    bool loadConfig(const std::string& modelPath, const std::string& configPath,
                    const std::string& calibrationPath, UdalsGeometry* outGeo);

    // Immutable after loadConfig() succeeds; safe to read from CwbClient without mLock.
    const UDalsStockModel& model() const { return mModel; }

    void reset();
    void clearSample();
    void setBacklight(uint32_t brightness);
    void setSampledRgb(const UdalsRgb& rgb);
    void updateTransitionHold(int8_t pendingDir, float uncompLux, float anchorUncompLux,
                              uint64_t expectedSeq);
    void clearTransitionHold();
    bool isUnambiguousAmbient(float uncompLux) const;
    float computeUncompensatedLux(const UdalsChannels& raw) const;
    std::optional<float> compensate(const UdalsChannels& raw);
    uint64_t getSampleSeq() const;
    uint32_t getBacklight() const;

  private:
    using SteadyClock = std::chrono::steady_clock;
    using TimePoint = SteadyClock::time_point;

    struct BacklightHoldEntry {
        TimePoint endTime;
        uint32_t brightness;
    };

    enum class HoldMode : uint8_t {
        kNone,
        kWake,
        kTransition,
    };

    struct HoldState {
        HoldMode mode = HoldMode::kNone;
        int8_t dir = 0;
        bool timedOut = false;
        float anchorUncompLux = 0.0f;
        TimePoint startTime{};
    };

    void pruneBacklightHistoryLocked(TimePoint now);
    uint32_t peakBacklightLocked(TimePoint now) const;
    uint32_t effectiveBacklightLocked(TimePoint now);
    float currentBleedLuxLocked(uint32_t effBkl) const;
    float maxWhiteBleedLuxLocked(uint32_t effBkl) const;
    bool isTransitionBleedPlausibleLocked(int8_t dir, uint32_t effBkl, float uncompLux,
                                          float anchorUncompLux) const;

    mutable std::mutex mLock;
    UDalsStockModel mModel;
    StockLuxFilter mFilter;
    uint32_t mBacklight = 0;
    uint64_t mSampleSeq = 0;
    HoldState mHold{};
    std::optional<UdalsRgb> mSampledRgb;
    std::deque<BacklightHoldEntry> mBacklightHistory;
};
