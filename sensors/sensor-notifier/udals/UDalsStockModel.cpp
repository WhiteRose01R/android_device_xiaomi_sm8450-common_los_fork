/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "UDalsStockModel.h"

#include <json/json.h>
#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <utility>

namespace {

constexpr int kStockNormalMaxDbv = 2047;
constexpr double kMaxHbmScale = 3.0;
constexpr int32_t kMaxGridDim = 64;
constexpr int32_t kMaxBlockSize = 32;
constexpr int32_t kMaxGridPixels = 512;
constexpr size_t kMaxNetworks = 16;
constexpr size_t kMaxLayers = 8;
constexpr size_t kMaxLayerWidth = 1024;
constexpr size_t kMaxFilterWindow = 32;
constexpr size_t kMaxDeadbandTiers = 16;

bool parseJsonString(const std::string& content, Json::Value* value, std::string* error) {
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    return reader->parse(content.data(), content.data() + content.size(), value, error);
}

bool readFileContent(const std::string& path, std::string* out, std::string* error) {
    std::ifstream input(path);
    if (!input) {
        if (error != nullptr) {
            *error = "Cannot read " + path;
        }
        return false;
    }
    std::ostringstream ss;
    ss << input.rdbuf();
    *out = ss.str();
    return true;
}

bool parseNumberList(const Json::Value& value, std::vector<double>* output) {
    std::vector<double> result;
    if (value.isString()) {
        std::string raw = value.asString();
        while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.back()))) {
            raw.pop_back();
        }
        if (!raw.empty() && raw.back() == ',') {
            raw.pop_back();
            while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.back()))) {
                raw.pop_back();
            }
            if (!raw.empty() && raw.back() == ',') {
                return false;
            }
        }
        std::istringstream stream(raw);
        std::string column;
        while (std::getline(stream, column, ',')) {
            std::istringstream number(column);
            double v = 0.0;
            if (!(number >> v) || !std::isfinite(v)) {
                return false;
            }
            number >> std::ws;
            if (!number.eof()) {
                return false;
            }
            result.push_back(v);
        }
    } else if (value.isArray()) {
        result.reserve(value.size());
        for (const auto& column : value) {
            if (!column.isNumeric() || !std::isfinite(column.asDouble())) {
                return false;
            }
            result.push_back(column.asDouble());
        }
    } else {
        return false;
    }
    *output = std::move(result);
    return true;
}

bool parseNumbers(const Json::Value& value, size_t count, std::vector<double>* output) {
    std::vector<double> result;
    if (!parseNumberList(value, &result) || result.size() != count) {
        return false;
    }
    *output = std::move(result);
    return true;
}

bool parseChannels(const Json::Value& value, UdalsChannels* output) {
    std::vector<double> values;
    if (!parseNumbers(value, 4, &values)) {
        return false;
    }
    std::copy(values.begin(), values.end(), output->begin());
    return true;
}

bool parseOptionalDouble(const Json::Value& obj, const char* key, double* out) {
    if (!obj.isMember(key)) {
        return true;
    }
    if (!obj[key].isNumeric() || !std::isfinite(obj[key].asDouble())) {
        return false;
    }
    *out = obj[key].asDouble();
    return true;
}

bool parseOptionalBool(const Json::Value& obj, const char* key, bool* out) {
    if (!obj.isMember(key)) {
        return true;
    }
    if (!obj[key].isBool()) {
        return false;
    }
    *out = obj[key].asBool();
    return true;
}

bool parseOptionalNonNegInt(const Json::Value& obj, const char* key, int* out) {
    if (!obj.isMember(key)) {
        return true;
    }
    if (!obj[key].isInt() || obj[key].asInt() < 0) {
        return false;
    }
    *out = obj[key].asInt();
    return true;
}

}  // namespace

void StockLuxFilter::configure(const StockLuxFilterConfig& config) {
    mConfig = config;
    reset();
}

std::optional<float> StockLuxFilter::apply(float value) {
    if (!std::isfinite(value)) {
        return std::nullopt;
    }
    if (mConfig.lowLuxWindow > 0) {
        if (mLowLux.empty()) {
            mLowLux.push_back(static_cast<double>(value));
        } else {
            const size_t count = mLowLux.size();
            const double mean = std::accumulate(mLowLux.begin(), mLowLux.end(), 0.0) /
                                static_cast<double>(count);
            const double valDouble = static_cast<double>(value);
            if ((mean < 8.0 && std::abs(valDouble - mean) <= 4.0) ||
                (mean >= 8.0 && mean < 20.0 && std::abs(valDouble - mean) <= 8.0)) {
                if (count == mConfig.lowLuxWindow) {
                    mLowLux.pop_front();
                }
                mLowLux.push_back(valDouble);
                // Average the previous count, excluding the new element until the window is full.
                value = static_cast<float>(
                        std::accumulate(mLowLux.begin(),
                                        mLowLux.begin() + static_cast<std::ptrdiff_t>(count), 0.0) /
                        static_cast<double>(count));
            } else {
                mLowLux.clear();
            }
        }
    }
    if (mConfig.medianWindow > 1) {
        mMedian.push_back(static_cast<double>(value));
        if (mMedian.size() > mConfig.medianWindow) {
            mMedian.pop_front();
        }
        if (mMedian.size() == mConfig.medianWindow) {
            auto sorted = mMedian;
            std::sort(sorted.begin(), sorted.end());
            const double median = sorted[mConfig.medianWindow / 2];
            value = mConfig.truncateMedianToInt ? static_cast<float>(std::trunc(median))
                                                : static_cast<float>(median);
        }
    }
    if (mLastReported && !mConfig.deadbandTiers.empty()) {
        const double previous = static_cast<double>(*mLastReported);
        double threshold = mConfig.deadbandTiers.back().ratio;
        for (const auto& tier : mConfig.deadbandTiers) {
            if (!tier.below || previous < *tier.below) {
                threshold = tier.ratio;
                break;
            }
        }
        const double difference = std::abs(static_cast<double>(value) - previous);
        const bool suppressed = mConfig.deadbandStrictLess ? (difference < previous * threshold)
                                                           : (difference <= previous * threshold);
        if (suppressed) {
            return std::nullopt;
        }
    }
    return value;
}

void StockLuxFilter::reset() {
    mLowLux.clear();
    mMedian.clear();
    mLastReported.reset();
}

bool UDalsStockModel::load(const std::string& modelPath, const std::string& configPath,
                           const std::string& calibrationPath, std::string* error,
                           std::string* warning) {
    std::string modelJson;
    std::string configJson;
    if (!readFileContent(modelPath, &modelJson, error) ||
        !readFileContent(configPath, &configJson, error)) {
        return false;
    }

    std::optional<std::string> calibrationJson;
    if (!calibrationPath.empty()) {
        struct stat calStat{};
        if (stat(calibrationPath.c_str(), &calStat) == 0) {
            std::string calErr;
            std::string content;
            if (readFileContent(calibrationPath, &content, &calErr)) {
                calibrationJson = std::move(content);
            } else if (warning != nullptr) {
                *warning = "Cannot read per-unit calibration " + calibrationPath + " (" + calErr +
                           "); using unity calibration";
            }
        } else if (errno != ENOENT && warning != nullptr) {
            *warning = "Cannot access per-unit calibration " + calibrationPath + " (" +
                       std::strerror(errno) + "); using unity calibration";
        }
    }

    return loadFromStrings(modelJson, configJson, calibrationJson, error, warning);
}

bool UDalsStockModel::loadFromStrings(const std::string& modelJson, const std::string& configJson,
                                      const std::optional<std::string>& calibrationJson,
                                      std::string* error, std::string* warning) {
    auto setError = [error](const std::string& msg) {
        if (error != nullptr) {
            *error = msg;
        }
    };

    // Commit only a complete validated configuration; failed reloads cannot mix profiles.
    UDalsStockModel parsed;
    Json::Value model;
    Json::Value config;
    std::string parseErr;
    if (!parseJsonString(modelJson, &model, &parseErr)) {
        setError("Failed to parse UDALS model JSON: " + parseErr);
        return false;
    }
    if (!parseJsonString(configJson, &config, &parseErr)) {
        setError("Failed to parse light sensor config JSON: " + parseErr);
        return false;
    }

    setError("Invalid UDALS model or calibration");
    if (!model.isObject() || !config.isObject() || model["version"] != 1 ||
        !model["grid"].isInt() || !model["block_size"].isInt() || !model["gamma"].isNumeric()) {
        return false;
    }

    parsed.mGrid = model["grid"].asInt();
    parsed.mBlockSize = model["block_size"].asInt();
    if (parsed.mGrid < 1 || parsed.mGrid > kMaxGridDim || parsed.mBlockSize < 1 ||
        parsed.mBlockSize > kMaxBlockSize || parsed.mGrid * parsed.mBlockSize > kMaxGridPixels) {
        return false;
    }

    double msgUnknown1 = 2.0;
    if (!parseOptionalDouble(model, "msg_unknown1", &msgUnknown1) || msgUnknown1 < 0.0) {
        return false;
    }
    parsed.mDefaultMsgUnknown1 = static_cast<float>(msgUnknown1);

    const double gamma = model["gamma"].asDouble();
    if (!std::isfinite(gamma) || gamma < 1.0 || gamma > 4.0) {
        return false;
    }
    for (size_t i = 0; i < parsed.mGamma.size(); ++i) {
        parsed.mGamma[i] = std::pow(static_cast<double>(i), gamma);
    }

    const size_t gridCells = static_cast<size_t>(parsed.mGrid) * static_cast<size_t>(parsed.mGrid);
    if (!model["spatial"].isArray() || model["spatial"].size() != 3) {
        return false;
    }
    for (Json::ArrayIndex c = 0; c < 3; ++c) {
        if (!parseNumbers(model["spatial"][c], gridCells, &parsed.mSpatial[c])) {
            return false;
        }
    }

    if (!config["Als"].isObject()) {
        return false;
    }
    const auto& als = config["Als"];
    if (!als["panel_Info_cali_ori"].isObject() || !als["channel_coef"].isObject() ||
        !als["cwbInfo"].isObject() || !als["panel_Info_cali_ori"]["owner"].isString() ||
        !als["channel_coef"]["owner"].isString()) {
        return false;
    }

    const std::string configOwner = als["panel_Info_cali_ori"]["owner"].asString();
    if (configOwner.empty() || als["channel_coef"]["owner"].asString() != configOwner) {
        return false;
    }
    if (model.isMember("sensor_owner")) {
        if (!model["sensor_owner"].isString() || model["sensor_owner"].asString() != configOwner) {
            return false;
        }
    }
    parsed.mSensorOwner = configOwner;

    std::vector<double> center;
    std::vector<double> radii;
    if (!parseNumbers(als["cwbInfo"]["Center"], 2, &center) ||
        !parseNumbers(als["cwbInfo"]["Radius"], 3, &radii)) {
        return false;
    }

    const int32_t gridSize = parsed.mGrid * parsed.mBlockSize;
    if (radii[2] * 2.0 + 1.0 != static_cast<double>(gridSize) || radii[0] <= 0.0 ||
        radii[1] <= radii[0] || radii[2] <= radii[1]) {
        return false;
    }
    if (center[0] != std::floor(center[0]) || center[1] != std::floor(center[1]) ||
        center[0] < radii[2] || center[1] < radii[2] || center[0] > 8192.0 || center[1] > 8192.0) {
        return false;
    }

    const int32_t gridLeft = static_cast<int32_t>(center[0] - radii[2]);
    const int32_t gridTop = static_cast<int32_t>(center[1] - radii[2]);
    // Align CWB ROI to an even origin with dimensions divisible by 8 pixels so
    // width * height * 4 (RGBA_8888) stays 256-byte aligned for SDM without crop expansion.
    const int32_t roiLeft = gridLeft & ~1;
    const int32_t roiTop = gridTop & ~1;
    const int32_t roiWidth = (gridLeft + gridSize - roiLeft + 7) & ~7;
    const int32_t roiHeight = (gridTop + gridSize - roiTop + 7) & ~7;
    parsed.mGeometry = {
            .gridLeft = gridLeft,
            .gridTop = gridTop,
            .gridWidth = gridSize,
            .gridHeight = gridSize,
            .roiLeft = roiLeft,
            .roiTop = roiTop,
            .roiRight = roiLeft + roiWidth,
            .roiBottom = roiTop + roiHeight,
    };

    if (!parseNumberList(als["channel_coef"]["coef_data"], &parsed.mCoefficients) ||
        parsed.mCoefficients.size() < 5) {
        return false;
    }

    auto parseCurve = [](const Json::Value& value, Curve* output) {
        if (!value.isArray() || value.size() < 2 || value.size() > 256) {
            return false;
        }
        Curve result;
        result.reserve(value.size());
        for (const auto& row : value) {
            std::vector<double> columns;
            if (!parseNumbers(row, 5, &columns) || columns[0] < 0.0 || columns[0] > 4095.0 ||
                columns[0] != std::floor(columns[0]) ||
                (!result.empty() && static_cast<int>(columns[0]) <= result.back().dbv)) {
                return false;
            }
            Point point{static_cast<int>(columns[0]), {}};
            for (size_t i = 0; i < 4; ++i) {
                if (columns[i + 1] < 0.0) {
                    return false;
                }
                point.channels[i] = columns[i + 1];
            }
            result.push_back(point);
        }
        *output = std::move(result);
        return true;
    };

    if (!parseCurve(als["panel_Info_cali_ori"]["data_w"], &parsed.mOriginal)) {
        return false;
    }
    parsed.mOriginalDc = parsed.mOriginal;
    if (als["panel_Info_cali_ori_dc"].isObject() &&
        als["panel_Info_cali_ori_dc"]["owner"] == configOwner) {
        if (!parseCurve(als["panel_Info_cali_ori_dc"]["data_w"], &parsed.mOriginalDc)) {
            return false;
        }
    }

    // Per-unit persist calibration is optional; malformed persist data falls back to unity
    // calibration with a warning rather than disabling bleed compensation completely.
    if (calibrationJson) {
        Json::Value calibration;
        std::string calErr;
        bool calValid = parseJsonString(*calibrationJson, &calibration, &calErr) &&
                        calibration.isObject() && calibration["Als"].isObject();
        Curve calNormal;
        Curve calDc;
        UdalsChannels calScale{1.0, 1.0, 1.0, 1.0};
        if (calValid) {
            const auto& cal = calibration["Als"];
            if (cal.isMember("panel_Info_cali")) {
                calValid = cal["panel_Info_cali"].isObject() &&
                           parseCurve(cal["panel_Info_cali"]["data_w"], &calNormal);
            }
            if (calValid && cal.isMember("panel_Info_cali_dc")) {
                calValid = cal["panel_Info_cali_dc"].isObject() &&
                           parseCurve(cal["panel_Info_cali_dc"]["data_w"], &calDc);
            }
            if (calValid && cal.isMember("channel_scale")) {
                calValid = cal["channel_scale"].isObject() &&
                           parseChannels(cal["channel_scale"]["scale_data"], &calScale);
                for (double scale : calScale) {
                    if (scale <= 0.0) {
                        calValid = false;
                    }
                }
            }
        }
        if (calValid) {
            parsed.mCalibrated = std::move(calNormal);
            parsed.mCalibratedDc = std::move(calDc);
            parsed.mScale = calScale;
        } else if (warning != nullptr) {
            *warning = "Invalid per-unit calibration JSON" +
                       (calErr.empty() ? "" : (": " + calErr)) + "; using unity calibration";
        }
    }

    if (!model["input_transform"].isString()) {
        return false;
    }
    const std::string transform = model["input_transform"].asString();
    if (transform == "affine") {
        parsed.mInputTransform = InputTransform::kAffine;
        if (!parseChannels(model["input_offset"], &parsed.mOffset) ||
            !parseChannels(model["input_divisor"], &parsed.mDivisor)) {
            return false;
        }
        for (double divisor : parsed.mDivisor) {
            if (divisor <= 0.0) {
                return false;
            }
        }
    } else if (transform == "log") {
        parsed.mInputTransform = InputTransform::kLog;
    } else {
        return false;
    }

    if (!model["routing"].isString()) {
        return false;
    }
    const std::string routing = model["routing"].asString();
    if (routing == "per_channel") {
        parsed.mRouting = Routing::kPerChannel;
    } else if (routing == "dbv_rules") {
        parsed.mRouting = Routing::kDbvRules;
    } else {
        return false;
    }

    if (!parseOptionalNonNegInt(model, "dc_curve_max_dbv", &parsed.mDcCurveMaxDbv)) {
        return false;
    }

    if (!model["networks"].isArray() || model["networks"].empty() ||
        model["networks"].size() > kMaxNetworks) {
        return false;
    }
    if (parsed.mRouting == Routing::kPerChannel && model["networks"].size() != 4) {
        return false;
    }
    const size_t expectedOutputs = (parsed.mRouting == Routing::kPerChannel) ? 1u : 4u;
    bool hasCatchAllNormal = false;
    bool hasCatchAllDc = false;
    for (const auto& network : model["networks"]) {
        if (!network.isObject()) {
            return false;
        }
        Network net;
        if (network.isMember("max_dbv") && !network["max_dbv"].isNull()) {
            if (!network["max_dbv"].isInt() || network["max_dbv"].asInt() < 0) {
                return false;
            }
            net.maxDbv = network["max_dbv"].asInt();
        }
        if (network.isMember("dc") && !network["dc"].isNull()) {
            if (!network["dc"].isBool()) {
                return false;
            }
            net.dc = network["dc"].asBool();
        }
        if (!net.maxDbv) {
            if (!net.dc || !*net.dc) {
                hasCatchAllNormal = true;
            }
            if (!net.dc || *net.dc) {
                hasCatchAllDc = true;
            }
        }
        if (!parseNumbers(network["exponent"], expectedOutputs, &net.exponent) ||
            !parseNumbers(network["divisor"], expectedOutputs, &net.divisor)) {
            return false;
        }
        for (double divisor : net.divisor) {
            if (divisor <= 0.0) {
                return false;
            }
        }
        if (!network["negative"].isString()) {
            return false;
        }
        const std::string negative = network["negative"].asString();
        if (negative == "allow") {
            net.negative = Network::Negative::kAllow;
        } else if (negative == "zero") {
            net.negative = Network::Negative::kZero;
        } else if (negative == "clamp") {
            net.negative = Network::Negative::kClamp;
        } else {
            return false;
        }
        if (!network["layers"].isArray() || network["layers"].empty() ||
            network["layers"].size() > kMaxLayers) {
            return false;
        }
        size_t inputs = 4;
        const Json::ArrayIndex layerCount = network["layers"].size();
        for (Json::ArrayIndex index = 0; index < layerCount; ++index) {
            const auto& layer = network["layers"][index];
            if (!layer.isObject() || !layer["outputs"].isUInt() || !layer["inputs"].isUInt() ||
                layer["inputs"].asUInt() != inputs) {
                return false;
            }
            const size_t width = layer["outputs"].asUInt();
            if (width == 0 || width > kMaxLayerWidth ||
                (index + 1 == layerCount && width != expectedOutputs)) {
                return false;
            }
            Layer dense{inputs, width, {}, {}};
            if (!parseNumbers(layer["weights"], inputs * width, &dense.weights) ||
                !parseNumbers(layer["bias"], width, &dense.bias)) {
                return false;
            }
            net.layers.push_back(std::move(dense));
            inputs = width;
        }
        parsed.mNetworks.push_back(std::move(net));
    }
    if (parsed.mRouting == Routing::kDbvRules && (!hasCatchAllNormal || !hasCatchAllDc)) {
        return false;
    }

    if (model.isMember("lux")) {
        const auto& lux = model["lux"];
        if (!lux.isObject() || !parseOptionalBool(lux, "coef_scale", &parsed.mLux.coefScale) ||
            !parseOptionalDouble(lux, "divisor", &parsed.mLux.divisor) ||
            !parseOptionalDouble(lux, "saturation_raw", &parsed.mLux.saturationRaw) ||
            !parseOptionalDouble(lux, "saturation_lux", &parsed.mLux.saturationLux) ||
            !parseOptionalNonNegInt(lux, "dbv_offset_above", &parsed.mLux.dbvOffsetAbove) ||
            !parseOptionalDouble(lux, "dbv_offset", &parsed.mLux.dbvOffset) ||
            !parseOptionalDouble(lux, "low_lux_threshold", &parsed.mLux.lowLuxThreshold) ||
            !parseOptionalDouble(lux, "low_lux_offset", &parsed.mLux.lowLuxOffset)) {
            return false;
        }
        if (parsed.mLux.divisor <= 0.0 || parsed.mLux.saturationRaw < 0.0 ||
            parsed.mLux.saturationLux < 0.0 || parsed.mLux.lowLuxThreshold < 0.0) {
            return false;
        }
    }

    if (model.isMember("filter")) {
        const auto& flt = model["filter"];
        if (!flt.isObject()) {
            return false;
        }
        int lowLuxWindow = 0;
        int medianWindow = 0;
        if (!parseOptionalNonNegInt(flt, "low_lux_window", &lowLuxWindow) ||
            !parseOptionalNonNegInt(flt, "median_window", &medianWindow) ||
            !parseOptionalBool(flt, "truncate_median_to_int",
                               &parsed.mFilterConfig.truncateMedianToInt) ||
            !parseOptionalBool(flt, "deadband_strict_less",
                               &parsed.mFilterConfig.deadbandStrictLess)) {
            return false;
        }
        if (static_cast<size_t>(lowLuxWindow) > kMaxFilterWindow ||
            static_cast<size_t>(medianWindow) > kMaxFilterWindow) {
            return false;
        }
        parsed.mFilterConfig.lowLuxWindow = static_cast<size_t>(lowLuxWindow);
        parsed.mFilterConfig.medianWindow = static_cast<size_t>(medianWindow);

        if (flt.isMember("deadband_tiers")) {
            const auto& tiers = flt["deadband_tiers"];
            if (!tiers.isArray() || tiers.size() > kMaxDeadbandTiers) {
                return false;
            }
            for (const auto& tierVal : tiers) {
                if (!tierVal.isObject() || !tierVal["ratio"].isNumeric() ||
                    !std::isfinite(tierVal["ratio"].asDouble()) ||
                    tierVal["ratio"].asDouble() < 0.0) {
                    return false;
                }
                StockLuxFilterConfig::DeadbandTier tier;
                tier.ratio = tierVal["ratio"].asDouble();
                if (tierVal.isMember("below") && !tierVal["below"].isNull()) {
                    if (!tierVal["below"].isNumeric() ||
                        !std::isfinite(tierVal["below"].asDouble()) ||
                        tierVal["below"].asDouble() <= 0.0) {
                        return false;
                    }
                    tier.below = tierVal["below"].asDouble();
                }
                parsed.mFilterConfig.deadbandTiers.push_back(tier);
            }
        }
    }

    const double pixelsPerBlock = static_cast<double>(parsed.mBlockSize * parsed.mBlockSize);
    for (size_t c = 0; c < 3; ++c) {
        const double weightSum =
                std::accumulate(parsed.mSpatial[c].begin(), parsed.mSpatial[c].end(), 0.0);
        parsed.mMaxWhiteRgb[c] = parsed.mGamma[255] * pixelsPerBlock * weightSum;
    }

    parsed.mLoaded = true;
    *this = std::move(parsed);
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

std::vector<double> UDalsStockModel::Network::evaluate(const UdalsChannels& input) const {
    std::vector<double> values(input.begin(), input.end());
    for (size_t l = 0; l < layers.size(); ++l) {
        const auto& layer = layers[l];
        std::vector<double> next(layer.outputs);
        for (size_t j = 0; j < layer.outputs; ++j) {
            double sum = 0.0;
            for (size_t i = 0; i < layer.inputs; ++i) {
                sum += values[i] * layer.weights[i * layer.outputs + j];
            }
            sum += layer.bias[j];
            next[j] = (l + 1 == layers.size()) ? sum : std::max(0.0, sum);
        }
        values = std::move(next);
    }
    for (size_t i = 0; i < values.size(); ++i) {
        const double output = (std::pow(10.0, exponent[i] - values[i] / 100.0) - 1.0) / divisor[i];
        values[i] = (negative == Negative::kZero && values[i] < 0.0) ? 0.0 : output;
        if (negative == Negative::kClamp) {
            values[i] = std::max(0.0, values[i]);
        }
    }
    return values;
}

UdalsRgb UDalsStockModel::samplePixels(const uint8_t* rgba, size_t stridePixels, int32_t bufLeft,
                                       int32_t bufTop) const {
    UdalsRgb sum{};
    if (!mLoaded || rgba == nullptr || mGrid <= 0 || mBlockSize <= 0 ||
        mGeometry.gridLeft < bufLeft || mGeometry.gridTop < bufTop) {
        return sum;
    }
    const size_t offsetX = static_cast<size_t>(mGeometry.gridLeft - bufLeft);
    const size_t offsetY = static_cast<size_t>(mGeometry.gridTop - bufTop);
    const size_t grid = static_cast<size_t>(mGrid);
    const size_t blockSize = static_cast<size_t>(mBlockSize);
    // Visit blocks in row order, then pixels in column order to preserve exact accumulation order.
    for (size_t by = 0; by < grid; ++by) {
        for (size_t bx = 0; bx < grid; ++bx) {
            const size_t block = by * grid + bx;
            for (size_t x = 0; x < blockSize; ++x) {
                for (size_t y = 0; y < blockSize; ++y) {
                    const size_t py = offsetY + by * blockSize + y;
                    const size_t px = offsetX + bx * blockSize + x;
                    const uint8_t* pixel = rgba + 4 * (py * stridePixels + px);
                    for (size_t c = 0; c < 3; ++c) {
                        sum[c] += mGamma[pixel[c]] * mSpatial[c][block];
                    }
                }
            }
        }
    }
    return sum;
}

UdalsChannels UDalsStockModel::interpolateCurve(const Curve& curve, int dbv) {
    if (dbv <= curve.front().dbv) {
        return curve.front().channels;
    }
    if (dbv >= curve.back().dbv) {
        return curve.back().channels;
    }
    auto upper = std::upper_bound(curve.begin(), curve.end(), dbv,
                                  [](int value, const Point& point) { return value < point.dbv; });
    const auto& lower = *(upper - 1);
    UdalsChannels result{};
    const double frac =
            static_cast<double>(dbv - lower.dbv) / static_cast<double>(upper->dbv - lower.dbv);
    for (size_t i = 0; i < 4; ++i) {
        result[i] = lower.channels[i] + (upper->channels[i] - lower.channels[i]) * frac;
    }
    return result;
}

UdalsChannels UDalsStockModel::predictBleed(int dbv, bool dc, const UdalsRgb& rgb) const {
    UdalsChannels result{};
    if (!mLoaded || dbv <= 0) {
        return result;
    }
    const int modelDbv = std::min(dbv, kStockNormalMaxDbv);
    UdalsChannels input{static_cast<double>(modelDbv), rgb[0], rgb[1], rgb[2]};
    if (mInputTransform == InputTransform::kAffine) {
        for (size_t i = 0; i < 4; ++i) {
            input[i] = (input[i] - mOffset[i]) / mDivisor[i];
        }
    } else {
        for (double& value : input) {
            value = std::log(value <= 1.0 ? value + 2.0 : value);
        }
    }

    if (mRouting == Routing::kPerChannel) {
        for (size_t i = 0; i < 4; ++i) {
            result[i] = mNetworks[i].evaluate(input)[0];
        }
    } else {
        const Network* selected = &mNetworks.back();
        for (const auto& net : mNetworks) {
            if ((!net.maxDbv || modelDbv <= *net.maxDbv) && (!net.dc || dc == *net.dc)) {
                selected = &net;
                break;
            }
        }
        auto predicted = selected->evaluate(input);
        std::copy(predicted.begin(), predicted.end(), result.begin());
    }

    const bool useDc = dc && mDcCurveMaxDbv > 0 && modelDbv <= mDcCurveMaxDbv;
    const auto& original = useDc ? mOriginalDc : mOriginal;
    const auto& calibrated = useDc ? mCalibratedDc : mCalibrated;
    // Apply per-unit calibration ratio only when measured and reference curves match.
    if (calibrated.size() == original.size() && !calibrated.empty() &&
        modelDbv > calibrated.front().dbv && modelDbv <= calibrated.back().dbv) {
        const auto reference = interpolateCurve(original, modelDbv);
        const auto measured = interpolateCurve(calibrated, modelDbv);
        for (size_t i = 0; i < 4; ++i) {
            if (reference[i] > 0.0) {
                result[i] *= measured[i] / reference[i];
            }
        }
    }

    // Extrapolate HBM (dbv > 2047) linearly from the peak normal-range stock prediction.
    if (dbv > kStockNormalMaxDbv) {
        const double hbmScale = std::min(
                static_cast<double>(dbv) / static_cast<double>(kStockNormalMaxDbv), kMaxHbmScale);
        for (double& ch : result) {
            ch *= hbmScale;
        }
    }
    return result;
}

float UDalsStockModel::computeLux(const UdalsChannels& raw, const UdalsChannels& bleed,
                                  int dbv) const {
    if (!mLoaded) {
        return 0.0f;
    }
    UdalsChannels corrected{};
    for (size_t i = 0; i < 4; ++i) {
        corrected[i] = std::max(0.0, raw[i] - bleed[i]) * mScale[i];
    }
    if (mLux.saturationRaw > 0.0 && (raw[0] > mLux.saturationRaw || raw[1] > mLux.saturationRaw ||
                                     raw[2] > mLux.saturationRaw)) {
        return static_cast<float>(mLux.saturationLux);
    }
    // Round each accumulated channel term to float to match stock evaluation.
    float value = 0.0f;
    for (size_t i = 0; i < 4; ++i) {
        value = static_cast<float>(static_cast<double>(value) +
                                   corrected[i] * mCoefficients[i + 1]);
    }
    if (mLux.coefScale) {
        value = static_cast<float>(static_cast<double>(value) * mCoefficients[0]);
    }
    if (mLux.divisor != 1.0) {
        value /= static_cast<float>(mLux.divisor);
    }
    if (mLux.dbvOffsetAbove > 0 && dbv > mLux.dbvOffsetAbove) {
        value += static_cast<float>(mLux.dbvOffset);
    }
    if (mLux.lowLuxThreshold > 0.0 && value < static_cast<float>(mLux.lowLuxThreshold)) {
        value += static_cast<float>(mLux.lowLuxOffset);
    }
    return std::max(0.0f, value);
}
