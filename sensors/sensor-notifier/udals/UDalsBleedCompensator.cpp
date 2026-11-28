/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsBleedCompensator"

#include "UDalsBleedCompensator.h"

#include <android-base/file.h>
#include <android-base/logging.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

namespace {

constexpr auto kWakeGateTimeout = std::chrono::milliseconds(250);
constexpr auto kTransitionHoldTimeout = std::chrono::milliseconds(250);
constexpr auto kBacklightRampDownHold = std::chrono::milliseconds(150);
constexpr size_t kMaxBacklightHistoryEntries = 64;
constexpr float kTransitionBleedCeilingFactor = 1.5f;
constexpr float kTransitionBleedCeilingMarginLux = 5.0f;

constexpr const char* kModelPaths[] = {
        "/odm/etc/sensors/udals-model.json",
        "/vendor/etc/sensors/udals-model.json",
};

constexpr const char* kConfigPaths[] = {
        "/odm/etc/sensors/config/lightSensorConfig.json",
        "/vendor/etc/sensors/config/lightSensorConfig.json",
};

constexpr const char kPersistCalibrationPath[] = "/mnt/vendor/persist/sensors/lightSensorCali.json";

float bleedCeilingLux(float bleed) {
    return bleed * kTransitionBleedCeilingFactor + kTransitionBleedCeilingMarginLux;
}

bool readFirstAvailable(const char* const* paths, size_t count, std::string* outContent,
                        std::string* outPath) {
    for (size_t i = 0; i < count; ++i) {
        if (android::base::ReadFileToString(paths[i], outContent)) {
            if (outPath != nullptr) {
                *outPath = paths[i];
            }
            return true;
        }
    }
    return false;
}

}  // namespace

bool UDalsBleedCompensator::loadConfig(UdalsGeometry* outGeo) {
    std::string modelJson;
    std::string modelPath;
    if (!readFirstAvailable(kModelPaths, std::size(kModelPaths), &modelJson, &modelPath)) {
        return false;
    }

    std::string configJson;
    std::string configPath;
    if (!readFirstAvailable(kConfigPaths, std::size(kConfigPaths), &configJson, &configPath)) {
        LOG(WARNING) << "Found UDALS model at " << modelPath
                     << " but no lightSensorConfig.json; disabling compensation";
        return false;
    }

    std::optional<std::string> calJson;
    std::string calContent;
    if (android::base::ReadFileToString(kPersistCalibrationPath, &calContent)) {
        calJson = std::move(calContent);
    } else if (errno != ENOENT) {
        PLOG(WARNING) << "Cannot read per-unit calibration " << kPersistCalibrationPath
                      << "; using unity calibration";
    }

    UDalsStockModel parsed;
    std::string error;
    std::string warning;
    if (!parsed.loadFromStrings(modelJson, configJson, calJson, &error, &warning)) {
        LOG(ERROR) << "Rejected UDALS stock configuration (" << modelPath << ", " << configPath
                   << "): " << error;
        return false;
    }
    if (!warning.empty()) {
        LOG(WARNING) << warning;
    }

    const UdalsGeometry geo = parsed.geometry();
    {
        std::lock_guard<std::mutex> lock(mLock);
        mModel = std::move(parsed);
        mFilter.configure(mModel.filterConfig());
    }
    if (outGeo != nullptr) {
        *outGeo = geo;
    }

    LOG(INFO) << "Loaded UDALS stock model (owner=" << mModel.sensorOwner() << ") ROI=["
              << geo.roiLeft << "," << geo.roiTop << "," << geo.roiRight << "," << geo.roiBottom
              << "] grid=[" << geo.gridLeft << "," << geo.gridTop << " " << geo.gridWidth << "x"
              << geo.gridHeight << "]";
    return true;
}

bool UDalsBleedCompensator::loadConfig(const std::string& modelPath, const std::string& configPath,
                                       const std::string& calibrationPath, UdalsGeometry* outGeo) {
    std::string modelJson;
    std::string configJson;
    if (!android::base::ReadFileToString(modelPath, &modelJson) ||
        !android::base::ReadFileToString(configPath, &configJson)) {
        return false;
    }

    std::optional<std::string> calJson;
    if (!calibrationPath.empty()) {
        std::string calContent;
        if (android::base::ReadFileToString(calibrationPath, &calContent)) {
            calJson = std::move(calContent);
        } else if (errno != ENOENT) {
            PLOG(WARNING) << "Cannot read per-unit calibration " << calibrationPath
                          << "; using unity calibration";
        }
    }

    UDalsStockModel parsed;
    std::string error;
    std::string warning;
    if (!parsed.loadFromStrings(modelJson, configJson, calJson, &error, &warning)) {
        LOG(ERROR) << "Rejected UDALS stock configuration: " << error;
        return false;
    }
    if (!warning.empty()) {
        LOG(WARNING) << warning;
    }

    const UdalsGeometry geo = parsed.geometry();
    {
        std::lock_guard<std::mutex> lock(mLock);
        mModel = std::move(parsed);
        mFilter.configure(mModel.filterConfig());
    }
    if (outGeo != nullptr) {
        *outGeo = geo;
    }
    return true;
}

void UDalsBleedCompensator::reset() {
    std::lock_guard<std::mutex> lock(mLock);
    mBacklight = 0;
    mSampledRgb.reset();
    mHold = {HoldMode::kWake, 0, false, 0.0f, SteadyClock::now()};
    ++mSampleSeq;
    mBacklightHistory.clear();
    mFilter.reset();
}

void UDalsBleedCompensator::clearSample() {
    std::lock_guard<std::mutex> lock(mLock);
    mSampledRgb.reset();
    if (mHold.mode != HoldMode::kWake) {
        mHold = {};
    }
    ++mSampleSeq;
    mFilter.reset();
}

void UDalsBleedCompensator::pruneBacklightHistoryLocked(TimePoint now) {
    while (!mBacklightHistory.empty() &&
           (now - mBacklightHistory.front().endTime) >= kBacklightRampDownHold) {
        mBacklightHistory.pop_front();
    }
}

uint32_t UDalsBleedCompensator::peakBacklightLocked(TimePoint now) const {
    if (mBacklight == 0) {
        return 0;
    }
    for (const auto& entry : mBacklightHistory) {
        if ((now - entry.endTime) < kBacklightRampDownHold) {
            return std::max(mBacklight, entry.brightness);
        }
    }
    return mBacklight;
}

uint32_t UDalsBleedCompensator::effectiveBacklightLocked(TimePoint now) {
    if (mBacklight == 0) {
        return 0;
    }
    pruneBacklightHistoryLocked(now);
    return peakBacklightLocked(now);
}

void UDalsBleedCompensator::setBacklight(uint32_t brightness) {
    std::lock_guard<std::mutex> lock(mLock);
    auto now = SteadyClock::now();
    pruneBacklightHistoryLocked(now);
    if (brightness < mBacklight && mBacklight > 0) {
        // Hold the higher pre-step brightness across the hardware ALS integration window.
        while (!mBacklightHistory.empty() && mBacklightHistory.back().brightness <= mBacklight) {
            mBacklightHistory.pop_back();
        }
        mBacklightHistory.push_back({now, mBacklight});
        while (mBacklightHistory.size() > kMaxBacklightHistoryEntries) {
            mBacklightHistory.pop_front();
        }
    }
    mBacklight = brightness;
}

uint64_t UDalsBleedCompensator::getSampleSeq() const {
    std::lock_guard<std::mutex> lock(mLock);
    return mSampleSeq;
}

uint32_t UDalsBleedCompensator::getBacklight() const {
    std::lock_guard<std::mutex> lock(mLock);
    return mBacklight;
}

void UDalsBleedCompensator::setSampledRgb(const UdalsRgb& rgb) {
    std::lock_guard<std::mutex> lock(mLock);
    mSampledRgb = rgb;
    mHold = {};
    ++mSampleSeq;
}

float UDalsBleedCompensator::currentBleedLuxLocked(uint32_t effBkl) const {
    if (effBkl == 0 || !mSampledRgb) {
        return 0.0f;
    }
    const auto bleed = mModel.predictBleed(static_cast<int>(effBkl), /*dc=*/false, *mSampledRgb);
    return mModel.computeLux(bleed, {}, /*dbv=*/0);
}

float UDalsBleedCompensator::maxWhiteBleedLuxLocked(uint32_t effBkl) const {
    if (effBkl == 0) {
        return 0.0f;
    }
    const auto whiteBleed =
            mModel.predictBleed(static_cast<int>(effBkl), /*dc=*/false, mModel.maxWhiteRgb());
    return mModel.computeLux(whiteBleed, {}, /*dbv=*/0);
}

bool UDalsBleedCompensator::isTransitionBleedPlausibleLocked(int8_t dir, uint32_t effBkl,
                                                             float uncompLux,
                                                             float anchorUncompLux) const {
    if (dir > 0) {
        return uncompLux <= bleedCeilingLux(maxWhiteBleedLuxLocked(effBkl));
    }
    if (dir < 0) {
        const float currentBleed = currentBleedLuxLocked(effBkl);
        if (currentBleed <= 0.0f) {
            return false;
        }
        return uncompLux < currentBleed ||
               (anchorUncompLux - uncompLux) <= bleedCeilingLux(currentBleed);
    }
    return false;
}

void UDalsBleedCompensator::updateTransitionHold(int8_t pendingDir, float uncompLux,
                                                 float anchorUncompLux, uint64_t expectedSeq) {
    std::lock_guard<std::mutex> lock(mLock);
    if (mHold.mode == HoldMode::kWake) {
        return;
    }
    if (pendingDir == 0) {
        mHold = {};
        return;
    }

    if (mSampleSeq != expectedSeq || !mSampledRgb || mBacklight == 0 || !mFilter.hasReported()) {
        return;
    }

    const int8_t normalizedDir = (pendingDir > 0) ? 1 : -1;
    if (mHold.mode == HoldMode::kTransition && mHold.dir != normalizedDir) {
        mHold = {};
    }

    auto now = SteadyClock::now();
    uint32_t effBkl = effectiveBacklightLocked(now);
    float effAnchor =
            (mHold.mode == HoldMode::kTransition) ? mHold.anchorUncompLux : anchorUncompLux;
    if (!isTransitionBleedPlausibleLocked(normalizedDir, effBkl, uncompLux, effAnchor)) {
        mHold = {};
        return;
    }

    if (mHold.mode == HoldMode::kNone && !mHold.timedOut) {
        mHold = {HoldMode::kTransition, normalizedDir, false, anchorUncompLux, now};
    }
}

void UDalsBleedCompensator::clearTransitionHold() {
    std::lock_guard<std::mutex> lock(mLock);
    if (mHold.mode != HoldMode::kWake) {
        mHold = {};
    }
}

bool UDalsBleedCompensator::isUnambiguousAmbient(float uncompLux) const {
    if (!std::isfinite(uncompLux) || uncompLux <= 0.0f) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mLock);
    uint32_t peak = peakBacklightLocked(SteadyClock::now());
    return uncompLux > bleedCeilingLux(maxWhiteBleedLuxLocked(peak));
}

float UDalsBleedCompensator::computeUncompensatedLux(const UdalsChannels& raw) const {
    for (double ch : raw) {
        if (!std::isfinite(ch) || ch < 0.0) {
            return -1.0f;
        }
    }
    // Use dbv = 0 so any high-DBV lux offset step does not inject a synthetic trigger jump.
    return mModel.computeLux(raw, {}, /*dbv=*/0);
}

std::optional<float> UDalsBleedCompensator::compensate(const UdalsChannels& raw) {
    for (double ch : raw) {
        if (!std::isfinite(ch) || ch < 0.0) {
            return std::nullopt;
        }
    }

    std::lock_guard<std::mutex> lock(mLock);
    auto now = SteadyClock::now();

    // Drop ALS events on wake until the first CWB capture completes or times out.
    if (mHold.mode == HoldMode::kWake) {
        if ((now - mHold.startTime) < kWakeGateTimeout) {
            return std::nullopt;
        }
        mHold = {};
    }

    const uint32_t effBkl = effectiveBacklightLocked(now);

    // Suppress emission while a bleed-plausible CWB capture is in flight so SSC holds steady
    // without polluting the stock filter window.
    if (mHold.mode == HoldMode::kTransition) {
        const auto elapsed = now - mHold.startTime;
        const float uncompLux = mModel.computeLux(raw, {}, /*dbv=*/0);
        if (mSampledRgb && effBkl > 0 && mFilter.hasReported() &&
            isTransitionBleedPlausibleLocked(mHold.dir, effBkl, uncompLux, mHold.anchorUncompLux) &&
            elapsed < kTransitionHoldTimeout) {
            return std::nullopt;
        }
        const bool timedOut = (elapsed >= kTransitionHoldTimeout);
        mHold = {HoldMode::kNone, 0, timedOut, 0.0f, {}};
    }

    const UdalsChannels bleed =
            (mSampledRgb && effBkl > 0)
                    ? mModel.predictBleed(static_cast<int>(effBkl), /*dc=*/false, *mSampledRgb)
                    : UdalsChannels{};
    const float lux = mModel.computeLux(raw, bleed, static_cast<int>(effBkl));
    auto filtered = mFilter.apply(lux);
    if (filtered) {
        mFilter.markReported(*filtered);
    }
    return filtered;
}
