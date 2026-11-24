/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UDalsBleedCompensator"

#include "UDalsBleedCompensator.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parsedouble.h>
#include <android-base/parseint.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <json/json.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace {

constexpr auto kWakeGateTimeout = std::chrono::milliseconds(250);
constexpr auto kTransitionHoldTimeout = std::chrono::milliseconds(250);
constexpr auto kBacklightRampDownHold = std::chrono::milliseconds(150);
constexpr size_t kMaxBacklightHistoryEntries = 64;
constexpr float kTransitionBleedCeilingFactor = 1.5f;
constexpr float kTransitionBleedCeilingMarginLux = 5.0f;
constexpr float kLowGammaKneeFraction = 0.025f;
constexpr size_t kResidualMedianWindow = 5;
constexpr float kLowLuxFilterCeiling = 5.0f;
constexpr float kLowLuxEmaCeiling = 3.0f;
constexpr float kEmaStepBypassDelta = 1.5f;
constexpr float kLowLuxEmaTauSeconds = 0.35f;

}  // namespace

UDalsBleedCompensator::UDalsBleedCompensator()
    : mBacklight(0),
      mNormalMaxBacklight(2047u),
      mDcMode(false),
      mHasSample(false),
      mWakeGateActive(false),
      mTransitionHoldDir(0),
      mTransitionHoldTimedOut(false),
      mHasSettledOutput(false),
      mHasResidualEma(false),
      mSampleSeq(0),
      mNormR(0.0f),
      mNormG(0.0f),
      mNormB(0.0f),
      mNormLowR(0.0f),
      mNormLowG(0.0f),
      mNormLowB(0.0f),
      mLastSettledCompLux(0.0f),
      mTransitionAnchorRawLux(0.0f),
      mResidualEma(0.0f),
      mPeakBleedLuxRed(0.0f),
      mPeakBleedLuxGreen(0.0f),
      mPeakBleedLuxBlue(0.0f),
      mResetTime(std::chrono::steady_clock::now()) {
    std::string bleedProp =
            android::base::GetProperty("ro.vendor.sensors.notifier.udals.max_bleed_rgb", "");
    if (!bleedProp.empty()) {
        auto parts = android::base::Split(bleedProp, ",");
        float r = 0.0f, g = 0.0f, b = 0.0f;
        if (parts.size() == 3 && android::base::ParseFloat(android::base::Trim(parts[0]), &r) &&
            android::base::ParseFloat(android::base::Trim(parts[1]), &g) &&
            android::base::ParseFloat(android::base::Trim(parts[2]), &b) && std::isfinite(r) &&
            std::isfinite(g) && std::isfinite(b) && r > 0.0f && g > 0.0f && b > 0.0f) {
            mPeakBleedLuxRed = r;
            mPeakBleedLuxGreen = g;
            mPeakBleedLuxBlue = b;
            LOG(INFO) << "Configured custom UDALS max bleed RGB: " << r << ", " << g << ", " << b;
        }
    }
}

bool UDalsBleedCompensator::loadConfig(const std::string& path, UdalsGeometry* outGeo) {
    if (mPeakBleedLuxRed <= 0.0f || mPeakBleedLuxGreen <= 0.0f || mPeakBleedLuxBlue <= 0.0f) {
        LOG(INFO) << "ro.vendor.sensors.notifier.udals.max_bleed_rgb not configured; "
                  << "skipping CWB compensation";
        return false;
    }

    std::string content;
    if (!android::base::ReadFileToString(path, &content)) {
        LOG(INFO) << "No light sensor config found at " << path;
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string errs;
    if (!reader->parse(content.data(), content.data() + content.size(), &root, &errs)) {
        LOG(ERROR) << "Failed to parse " << path << ": " << errs;
        return false;
    }

    if (!root.isObject() || !root["Als"].isObject() || !root["Als"]["cwbInfo"].isObject()) {
        LOG(INFO) << "Config does not contain Als.cwbInfo; skipping CWB compensation";
        return false;
    }

    const auto& als = root["Als"];
    const auto& cwb = als["cwbInfo"];
    if (!cwb["Center"].isString() || !cwb["Radius"].isString()) {
        return false;
    }

    auto centerParts = android::base::Split(cwb["Center"].asString(), ",");
    auto radiusParts = android::base::Split(cwb["Radius"].asString(), ",");
    if (centerParts.size() != 2 || radiusParts.size() != 3) {
        LOG(ERROR) << "Invalid Center or Radius format in " << path;
        return false;
    }

    int cx = 0, cy = 0, r1 = 0, r2 = 0, r3 = 0;
    if (!android::base::ParseInt(android::base::Trim(centerParts[0]), &cx) ||
        !android::base::ParseInt(android::base::Trim(centerParts[1]), &cy) ||
        !android::base::ParseInt(android::base::Trim(radiusParts[0]), &r1) ||
        !android::base::ParseInt(android::base::Trim(radiusParts[1]), &r2) ||
        !android::base::ParseInt(android::base::Trim(radiusParts[2]), &r3) || cx <= 0 || cy <= 0 ||
        cx > 4096 || cy > 4096 || r1 <= 0 || r2 <= r1 || r3 <= r2 || r3 > 512) {
        LOG(ERROR) << "Failed to parse numeric geometry from " << path;
        return false;
    }

    outGeo->cx = cx;
    outGeo->cy = cy;
    outGeo->r1 = r1;
    outGeo->r2 = r2;
    outGeo->r3 = r3;

    int halfSpan = (r3 + 3) & ~3;
    outGeo->left = std::max(0, cx - halfSpan);
    outGeo->top = std::max(0, cy - halfSpan);
    outGeo->right = cx + halfSpan;
    outGeo->bottom = cy + halfSpan;

    // Align ROI dimensions to multiples of 8 so width * height * 4 (RGBA_8888)
    // stays divisible by 256 bytes for SDM even when left or top clamp to 0.
    outGeo->right = outGeo->left + ((outGeo->right - outGeo->left + 7) & ~7);
    outGeo->bottom = outGeo->top + ((outGeo->bottom - outGeo->top + 7) & ~7);

    auto parseCurve = [](const Json::Value& arr, size_t colIndex, std::vector<CaliPoint>* out) {
        out->clear();
        if (!arr.isArray()) return false;
        for (const auto& item : arr) {
            if (!item.isString()) {
                continue;
            }
            auto cols = android::base::Split(item.asString(), ",");
            float b = 0.0f, clear = 0.0f;
            if (cols.size() > colIndex &&
                android::base::ParseFloat(android::base::Trim(cols[0]), &b) &&
                android::base::ParseFloat(android::base::Trim(cols[colIndex]), &clear) &&
                std::isfinite(b) && std::isfinite(clear) && b >= 0.0f && clear >= 0.0f &&
                (out->empty() || b > out->back().brightness)) {
                out->push_back({b, clear});
            }
        }
        return !out->empty() && out->back().clear > 0.0f;
    };

    auto loadCurves = [&parseCurve](const Json::Value& obj, std::vector<CaliPoint>* outR,
                                    std::vector<CaliPoint>* outG, std::vector<CaliPoint>* outB) {
        bool okR = parseCurve(obj["data_r"], 1, outR) || parseCurve(obj["data_w"], 2, outR) ||
                   parseCurve(obj["data_w"], 1, outR);
        bool okG = parseCurve(obj["data_g"], 1, outG) || parseCurve(obj["data_w"], 3, outG) ||
                   parseCurve(obj["data_w"], 1, outG);
        bool okB = parseCurve(obj["data_b"], 1, outB) || parseCurve(obj["data_w"], 4, outB) ||
                   parseCurve(obj["data_w"], 1, outB);
        return okR && okG && okB;
    };

    if (!als["panel_Info_cali_ori"].isObject()) {
        LOG(ERROR) << "Missing panel_Info_cali_ori object";
        return false;
    }

    const auto& norm = als["panel_Info_cali_ori"];
    if (!loadCurves(norm, &mNormalDataR, &mNormalDataG, &mNormalDataB)) {
        LOG(ERROR) << "Failed to parse panel_Info_cali_ori tables";
        return false;
    }

    mNormalMaxBacklight = static_cast<uint32_t>(std::max({
            mNormalDataR.back().brightness,
            mNormalDataG.back().brightness,
            mNormalDataB.back().brightness,
    }));

    bool dcLoaded = false;
    if (als["panel_Info_cali_ori_dc"].isObject()) {
        const auto& dc = als["panel_Info_cali_ori_dc"];
        std::string normOwner = norm["owner"].isString() ? norm["owner"].asString() : "";
        std::string dcOwner = dc["owner"].isString() ? dc["owner"].asString() : "";
        if (normOwner.empty() || dcOwner == normOwner) {
            dcLoaded = loadCurves(dc, &mDcDataR, &mDcDataG, &mDcDataB);
        } else {
            LOG(INFO) << "Skipping panel_Info_cali_ori_dc: owner '" << dcOwner
                      << "' does not match normal owner '" << normOwner << "'";
        }
    }
    if (!dcLoaded) {
        mDcDataR = mNormalDataR;
        mDcDataG = mNormalDataG;
        mDcDataB = mNormalDataB;
    }

    LOG(INFO) << "Loaded UDALS calibration: ROI=[" << outGeo->left << "," << outGeo->top << ","
              << outGeo->right << "," << outGeo->bottom << "] radii=[" << r1 << "," << r2 << ","
              << r3 << "] maxBkl=" << mNormalMaxBacklight;

    return true;
}

void UDalsBleedCompensator::reset() {
    std::lock_guard<std::mutex> lock(mLock);
    mBacklight = 0;
    mHasSample = false;
    mWakeGateActive = true;
    mTransitionHoldDir = 0;
    mTransitionHoldTimedOut = false;
    mHasSettledOutput = false;
    mLastSettledCompLux = 0.0f;
    mTransitionAnchorRawLux = 0.0f;
    ++mSampleSeq;
    mNormR = 0.0f;
    mNormG = 0.0f;
    mNormB = 0.0f;
    mNormLowR = 0.0f;
    mNormLowG = 0.0f;
    mNormLowB = 0.0f;
    mBacklightHistory.clear();
    resetResidualFilterLocked();
    mResetTime = std::chrono::steady_clock::now();
}

void UDalsBleedCompensator::clearSample() {
    std::lock_guard<std::mutex> lock(mLock);
    mHasSample = false;
    mTransitionHoldDir = 0;
    mTransitionHoldTimedOut = false;
    mTransitionAnchorRawLux = 0.0f;
    ++mSampleSeq;
    mNormR = 0.0f;
    mNormG = 0.0f;
    mNormB = 0.0f;
    mNormLowR = 0.0f;
    mNormLowG = 0.0f;
    mNormLowB = 0.0f;
    resetResidualFilterLocked();
}

void UDalsBleedCompensator::pruneBacklightHistoryLocked(std::chrono::steady_clock::time_point now) {
    while (!mBacklightHistory.empty() &&
           (now - mBacklightHistory.front().endTime) >= kBacklightRampDownHold) {
        mBacklightHistory.pop_front();
    }
}

uint32_t UDalsBleedCompensator::effectiveBacklightLocked(
        std::chrono::steady_clock::time_point now) {
    if (mBacklight == 0) {
        return 0;
    }
    pruneBacklightHistoryLocked(now);
    uint32_t peak = mBacklight;
    for (const auto& entry : mBacklightHistory) {
        peak = std::max(peak, entry.brightness);
    }
    return peak;
}

void UDalsBleedCompensator::setBacklight(uint32_t brightness) {
    std::lock_guard<std::mutex> lock(mLock);
    auto now = std::chrono::steady_clock::now();
    pruneBacklightHistoryLocked(now);
    if (brightness < mBacklight && mBacklight > 0) {
        // Record when the previous higher backlight level ended so dimming ramps hold
        // that level across the ~100-150 ms hardware ALS photon integration window.
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

uint32_t UDalsBleedCompensator::getNormalMaxBacklight() const {
    std::lock_guard<std::mutex> lock(mLock);
    return mNormalMaxBacklight;
}

void UDalsBleedCompensator::setDcMode(bool dcMode) {
    std::lock_guard<std::mutex> lock(mLock);
    mDcMode = dcMode;
}

void UDalsBleedCompensator::setSampledRgb(float rHigh, float gHigh, float bHigh, float rLow,
                                          float gLow, float bLow) {
    std::lock_guard<std::mutex> lock(mLock);
    mNormR = std::clamp(rHigh, 0.0f, 1.0f);
    mNormG = std::clamp(gHigh, 0.0f, 1.0f);
    mNormB = std::clamp(bHigh, 0.0f, 1.0f);
    mNormLowR = std::clamp(rLow, 0.0f, 1.0f);
    mNormLowG = std::clamp(gLow, 0.0f, 1.0f);
    mNormLowB = std::clamp(bLow, 0.0f, 1.0f);
    mHasSample = true;
    mWakeGateActive = false;
    mTransitionHoldDir = 0;
    mTransitionHoldTimedOut = false;
    mTransitionAnchorRawLux = 0.0f;
    ++mSampleSeq;
}

bool UDalsBleedCompensator::isTransitionBleedPlausibleLocked(int8_t dir, uint32_t effBkl,
                                                             float rawLux,
                                                             float anchorRawLux) const {
    if (dir > 0) {
        float maxWhiteBleed = calculateMaxWhiteBleedLocked(effBkl);
        return rawLux <=
               maxWhiteBleed * kTransitionBleedCeilingFactor + kTransitionBleedCeilingMarginLux;
    }
    if (dir < 0) {
        float currentBleed = calculateBleedLuxLocked(effBkl);
        if (currentBleed <= 0.0f) {
            return false;
        }
        float drop = anchorRawLux - rawLux;
        return rawLux < currentBleed || drop <= currentBleed * kTransitionBleedCeilingFactor +
                                                        kTransitionBleedCeilingMarginLux;
    }
    return false;
}

void UDalsBleedCompensator::updateTransitionHold(int8_t pendingDir, float rawLux,
                                                 float anchorRawLux, uint64_t expectedSeq) {
    std::lock_guard<std::mutex> lock(mLock);
    if (pendingDir == 0) {
        mTransitionHoldDir = 0;
        mTransitionHoldTimedOut = false;
        mTransitionAnchorRawLux = 0.0f;
        return;
    }

    if (mSampleSeq != expectedSeq || !mHasSample || mBacklight == 0 || mWakeGateActive ||
        !mHasSettledOutput) {
        return;
    }

    int8_t normalizedDir = (pendingDir > 0) ? 1 : -1;
    if (mTransitionHoldDir != 0 && mTransitionHoldDir != normalizedDir) {
        mTransitionHoldDir = 0;
        mTransitionHoldTimedOut = false;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t effBkl = effectiveBacklightLocked(now);
    float effAnchor =
            (mTransitionHoldDir == normalizedDir) ? mTransitionAnchorRawLux : anchorRawLux;
    if (!isTransitionBleedPlausibleLocked(normalizedDir, effBkl, rawLux, effAnchor)) {
        mTransitionHoldDir = 0;
        mTransitionHoldTimedOut = false;
        mTransitionAnchorRawLux = 0.0f;
        return;
    }

    if (mTransitionHoldDir == 0 && !mTransitionHoldTimedOut) {
        mTransitionHoldDir = normalizedDir;
        mTransitionAnchorRawLux = anchorRawLux;
        mTransitionHoldStart = now;
    }
}

void UDalsBleedCompensator::clearTransitionHold() {
    std::lock_guard<std::mutex> lock(mLock);
    mTransitionHoldDir = 0;
    mTransitionHoldTimedOut = false;
    mTransitionAnchorRawLux = 0.0f;
}

bool UDalsBleedCompensator::isUnambiguousAmbient(float rawLux) const {
    if (!std::isfinite(rawLux) || rawLux <= 0.0f) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mLock);
    if (mBacklight == 0) {
        return rawLux > kTransitionBleedCeilingMarginLux;
    }
    auto now = std::chrono::steady_clock::now();
    uint32_t peak = mBacklight;
    for (const auto& entry : mBacklightHistory) {
        if ((now - entry.endTime) < kBacklightRampDownHold) {
            peak = std::max(peak, entry.brightness);
        }
    }
    float maxWhiteBleed = calculateMaxWhiteBleedLocked(peak);
    return rawLux >
           maxWhiteBleed * kTransitionBleedCeilingFactor + kTransitionBleedCeilingMarginLux;
}

float UDalsBleedCompensator::interpolateTable(const std::vector<CaliPoint>& table,
                                              float brightness) {
    if (table.empty() || brightness <= 0.0f) {
        return 0.0f;
    }
    if (brightness <= table.front().brightness) {
        if (table.front().brightness > 1e-4f) {
            return (brightness / table.front().brightness) * table.front().clear;
        }
        return table.front().clear;
    }
    if (brightness >= table.back().brightness) {
        return table.back().clear;
    }

    for (size_t i = 0; i < table.size() - 1; ++i) {
        if (brightness >= table[i].brightness && brightness <= table[i + 1].brightness) {
            float b0 = table[i].brightness;
            float b1 = table[i + 1].brightness;
            float delta = b1 - b0;
            float t = (delta > 1e-4f) ? (brightness - b0) / delta : 0.0f;
            return table[i].clear + t * (table[i + 1].clear - table[i].clear);
        }
    }
    return 0.0f;
}

float UDalsBleedCompensator::calculateBleedLuxForRgbLocked(uint32_t backlight, float r, float g,
                                                           float b) const {
    if (backlight == 0) {
        return 0.0f;
    }

    if (r <= 0.001f && g <= 0.001f && b <= 0.001f) {
        return 0.0f;
    }

    const auto& tableR = mDcMode ? mDcDataR : mNormalDataR;
    const auto& tableG = mDcMode ? mDcDataG : mNormalDataG;
    const auto& tableB = mDcMode ? mDcDataB : mNormalDataB;

    if (tableR.empty() || tableG.empty() || tableB.empty()) {
        return 0.0f;
    }

    float maxClearR = tableR.back().clear;
    float maxClearG = tableG.back().clear;
    float maxClearB = tableB.back().clear;

    float curveR = 0.0f;
    float curveG = 0.0f;
    float curveB = 0.0f;
    if (mNormalMaxBacklight > 0 && backlight > mNormalMaxBacklight) {
        float hbmScale = std::min(
                static_cast<float>(backlight) / static_cast<float>(mNormalMaxBacklight), 3.0f);
        curveR = (maxClearR > 0.0f) ? hbmScale : 0.0f;
        curveG = (maxClearG > 0.0f) ? hbmScale : 0.0f;
        curveB = (maxClearB > 0.0f) ? hbmScale : 0.0f;
    } else {
        float clearAtBrightnessR = interpolateTable(tableR, static_cast<float>(backlight));
        float clearAtBrightnessG = interpolateTable(tableG, static_cast<float>(backlight));
        float clearAtBrightnessB = interpolateTable(tableB, static_cast<float>(backlight));

        curveR = (maxClearR > 0.0f) ? (clearAtBrightnessR / maxClearR) : 0.0f;
        curveG = (maxClearG > 0.0f) ? (clearAtBrightnessG / maxClearG) : 0.0f;
        curveB = (maxClearB > 0.0f) ? (clearAtBrightnessB / maxClearB) : 0.0f;
    }

    float bleedR = r * curveR * mPeakBleedLuxRed;
    float bleedG = g * curveG * mPeakBleedLuxGreen;
    float bleedB = b * curveB * mPeakBleedLuxBlue;

    return std::max(0.0f, bleedR + bleedG + bleedB);
}

float UDalsBleedCompensator::calculateBleedLuxLocked(uint32_t effectiveBacklight) const {
    if (effectiveBacklight == 0 || !mHasSample) {
        return 0.0f;
    }

    // Blend high-BKL (gamma 2.0) and low-BKL (gamma 1.6) linear bases to model low-DBV shadow lift.
    float knee = (mNormalMaxBacklight > 0)
                         ? std::max(1.0f,
                                    kLowGammaKneeFraction * static_cast<float>(mNormalMaxBacklight))
                         : 51.0f;
    float r = static_cast<float>(effectiveBacklight) / knee;
    float lowBlend = 1.0f / (1.0f + r * std::sqrt(r));
    float effR = mNormR + lowBlend * (mNormLowR - mNormR);
    float effG = mNormG + lowBlend * (mNormLowG - mNormG);
    float effB = mNormB + lowBlend * (mNormLowB - mNormB);

    return calculateBleedLuxForRgbLocked(effectiveBacklight, effR, effG, effB);
}

float UDalsBleedCompensator::calculateMaxWhiteBleedLocked(uint32_t effectiveBacklight) const {
    return calculateBleedLuxForRgbLocked(effectiveBacklight, 1.0f, 1.0f, 1.0f);
}

void UDalsBleedCompensator::resetResidualFilterLocked() {
    mResidualWindow.clear();
    mHasResidualEma = false;
    mResidualEma = 0.0f;
    mLastResidualSampleTime = {};
}

float UDalsBleedCompensator::filterSignedResidualLocked(float signedRes, float bleed,
                                                        std::chrono::steady_clock::time_point now) {
    if (bleed <= 0.0f) {
        if (mHasResidualEma && std::fabs(signedRes - mResidualEma) > kEmaStepBypassDelta) {
            mResidualWindow.clear();
        }
        mResidualWindow.push_back(signedRes);
        while (mResidualWindow.size() > kResidualMedianWindow) {
            mResidualWindow.pop_front();
        }
        mResidualEma = signedRes;
        mHasResidualEma = true;
        mLastResidualSampleTime = now;
        return std::max(0.0f, signedRes);
    }

    // Pass bright ambient (>= 5 lux) through directly while keeping the median window current.
    if (mHasResidualEma && mResidualEma >= kLowLuxFilterCeiling &&
        signedRes >= kLowLuxFilterCeiling) {
        if (signedRes < mResidualEma - kEmaStepBypassDelta) {
            for (float& val : mResidualWindow) {
                val = std::min(val, signedRes);
            }
        }
        mResidualWindow.push_back(signedRes);
        while (mResidualWindow.size() > kResidualMedianWindow) {
            mResidualWindow.pop_front();
        }
        mResidualEma = signedRes;
        mLastResidualSampleTime = now;
        return std::max(0.0f, signedRes);
    }

    // Filter signed (raw - bleed) before clamping at 0 so symmetric PWM ripple cancels out.
    mResidualWindow.push_back(signedRes);
    while (mResidualWindow.size() > kResidualMedianWindow) {
        mResidualWindow.pop_front();
    }

    std::array<float, kResidualMedianWindow> sorted{};
    size_t n = mResidualWindow.size();
    for (size_t i = 0; i < n; ++i) {
        sorted[i] = mResidualWindow[i];
    }
    std::sort(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(n));
    float med = sorted[(n - 1) / 2];

    // Check EMA step-bypass on the median output so brief spikes stay rejected.
    if (!mHasResidualEma || std::fabs(med - mResidualEma) > kEmaStepBypassDelta ||
        med >= kLowLuxEmaCeiling || mLastResidualSampleTime.time_since_epoch().count() == 0) {
        mResidualEma = med;
        mHasResidualEma = true;
    } else {
        float dtSec = std::chrono::duration<float>(now - mLastResidualSampleTime).count();
        dtSec = std::clamp(dtSec, 0.005f, 2.0f);
        float alpha = 1.0f - std::exp(-dtSec / kLowLuxEmaTauSeconds);
        mResidualEma = alpha * med + (1.0f - alpha) * mResidualEma;
    }
    mLastResidualSampleTime = now;

    return std::max(0.0f, mResidualEma);
}

bool UDalsBleedCompensator::compensate(float rawLux, float* outLux) {
    if (outLux == nullptr || !std::isfinite(rawLux) || rawLux < 0.0f) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mLock);
    auto now = std::chrono::steady_clock::now();

    // Drop ALS events on wake until the first CWB capture completes or times out.
    if (mWakeGateActive) {
        if ((now - mResetTime) < kWakeGateTimeout) {
            return false;
        }
        mWakeGateActive = false;
    }

    uint32_t effBkl = effectiveBacklightLocked(now);

    // Hold the last settled lux while a bleed-plausible CWB capture is pending.
    if (mTransitionHoldDir != 0) {
        auto elapsed = now - mTransitionHoldStart;
        if (mHasSample && effBkl > 0 && mHasSettledOutput &&
            isTransitionBleedPlausibleLocked(mTransitionHoldDir, effBkl, rawLux,
                                             mTransitionAnchorRawLux) &&
            elapsed < kTransitionHoldTimeout) {
            *outLux = std::min(mLastSettledCompLux, rawLux);
            return true;
        }
        mTransitionHoldDir = 0;
        mTransitionAnchorRawLux = 0.0f;
        if (elapsed >= kTransitionHoldTimeout) {
            mTransitionHoldTimedOut = true;
        }
    }

    float bleed = calculateBleedLuxLocked(effBkl);
    float filtered = filterSignedResidualLocked(rawLux - bleed, bleed, now);
    mLastSettledCompLux = filtered;
    mHasSettledOutput = true;
    *outLux = filtered;
    return true;
}
