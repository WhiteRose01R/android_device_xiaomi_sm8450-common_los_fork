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
#include <cmath>
#include <memory>

UDalsBleedCompensator::UDalsBleedCompensator()
    : mBacklight(0),
      mNormalMaxBacklight(2047u),
      mDcMode(false),
      mHbmMode(false),
      mHasSample(false),
      mNormR(0.0f),
      mNormG(0.0f),
      mNormB(0.0f),
      mPeakBleedLuxRed(0.0f),
      mPeakBleedLuxGreen(0.0f),
      mPeakBleedLuxBlue(0.0f),
      mResetTime(std::chrono::steady_clock::now()),
      mLastSampleTime(),
      mMedianWindow{},
      mMedianCount(0),
      mMedianIndex(0) {
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
    mMedianWindow.fill(0.0f);
    mMedianCount = 0;
    mMedianIndex = 0;
    mBacklight = 0;
    mHbmMode = false;
    mHasSample = false;
    mNormR = 0.0f;
    mNormG = 0.0f;
    mNormB = 0.0f;
    mResetTime = std::chrono::steady_clock::now();
    mLastSampleTime = std::chrono::steady_clock::time_point{};
}

void UDalsBleedCompensator::setBacklight(uint32_t brightness) {
    std::lock_guard<std::mutex> lock(mLock);
    uint32_t clamped = std::min(brightness, mNormalMaxBacklight);
    if (mBacklight == 0 && clamped > 0 && !mHasSample) {
        mResetTime = std::chrono::steady_clock::now();
    }
    mBacklight = clamped;
}

bool UDalsBleedCompensator::hasSample() const {
    std::lock_guard<std::mutex> lock(mLock);
    return mHasSample;
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

void UDalsBleedCompensator::setHbmMode(bool hbm) {
    std::lock_guard<std::mutex> lock(mLock);
    mHbmMode = hbm;
}

void UDalsBleedCompensator::setSampledRgb(float r, float g, float b) {
    std::lock_guard<std::mutex> lock(mLock);
    mNormR = std::clamp(r, 0.0f, 1.0f);
    mNormG = std::clamp(g, 0.0f, 1.0f);
    mNormB = std::clamp(b, 0.0f, 1.0f);
    if (!mHasSample) {
        mMedianWindow.fill(0.0f);
        mMedianCount = 0;
        mMedianIndex = 0;
    }
    mHasSample = true;
    mLastSampleTime = std::chrono::steady_clock::now();
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

float UDalsBleedCompensator::calculateBleedLuxLocked() const {
    if (mBacklight == 0 || !mHasSample || mLastSampleTime.time_since_epoch().count() == 0) {
        return 0.0f;
    }

    auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - mLastSampleTime)
                         .count();
    float stalenessFactor = 1.0f;
    if (ageMs > 1500) {
        // Decay gradually rather than snapping to zero during temporary CWB lockouts
        // (e.g., DRM or virtual displays), fading completely out after 5 s.
        if (ageMs >= 6500) {
            return 0.0f;
        }
        stalenessFactor = 1.0f - static_cast<float>(ageMs - 1500) / 5000.0f;
    }

    // Calibration tables and max_bleed_rgb are characterized up to mNormalMaxBacklight
    uint32_t effectiveBacklight = mHbmMode ? mNormalMaxBacklight : mBacklight;

    if (mNormR <= 0.001f && mNormG <= 0.001f && mNormB <= 0.001f) {
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

    float clearAtBrightnessR = interpolateTable(tableR, static_cast<float>(effectiveBacklight));
    float clearAtBrightnessG = interpolateTable(tableG, static_cast<float>(effectiveBacklight));
    float clearAtBrightnessB = interpolateTable(tableB, static_cast<float>(effectiveBacklight));

    float curveR = (maxClearR > 0.0f) ? (clearAtBrightnessR / maxClearR) : 0.0f;
    float curveG = (maxClearG > 0.0f) ? (clearAtBrightnessG / maxClearG) : 0.0f;
    float curveB = (maxClearB > 0.0f) ? (clearAtBrightnessB / maxClearB) : 0.0f;

    float bleedR = mNormR * curveR * mPeakBleedLuxRed;
    float bleedG = mNormG * curveG * mPeakBleedLuxGreen;
    float bleedB = mNormB * curveB * mPeakBleedLuxBlue;

    return std::max(0.0f, (bleedR + bleedG + bleedB) * stalenessFactor);
}

bool UDalsBleedCompensator::compensate(float rawLux, float* outLux) {
    if (outLux == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mLock);

    // Suppress flare spikes during display wake prior to first CWB frame capture or timeout
    if (!mHasSample && mBacklight > 0) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - mResetTime)
                               .count();
        if (elapsed < 250) {
            return false;
        }
    }

    float bleed = calculateBleedLuxLocked();
    float compensated = std::max(0.0f, rawLux - bleed);

    mMedianWindow[mMedianIndex] = compensated;
    mMedianIndex = (mMedianIndex + 1) % mMedianWindow.size();
    if (mMedianCount < mMedianWindow.size()) {
        mMedianCount++;
    }

    auto temp = mMedianWindow;
    size_t medianIdx = (mMedianCount - 1) / 2;
    std::nth_element(temp.begin(), temp.begin() + medianIdx, temp.begin() + mMedianCount);
    *outLux = temp[medianIdx];
    return true;
}
