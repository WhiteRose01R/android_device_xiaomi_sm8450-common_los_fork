/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

using UdalsChannels = std::array<double, 4>;  // Clear, red, green, blue in sensor counts.
using UdalsRgb = std::array<double, 3>;       // Spatially weighted R, G, B sums.

struct UdalsGeometry {
    int32_t gridLeft = 0;
    int32_t gridTop = 0;
    int32_t gridWidth = 0;
    int32_t gridHeight = 0;
    int32_t roiLeft = 0;
    int32_t roiTop = 0;
    int32_t roiRight = 0;
    int32_t roiBottom = 0;
};

struct StockLuxFilterConfig {
    struct DeadbandTier {
        std::optional<double> below;  // Unset means catch-all tier.
        double ratio = 0.0;
    };

    size_t lowLuxWindow = 0;
    size_t medianWindow = 0;
    bool truncateMedianToInt = false;
    bool deadbandStrictLess = true;
    std::vector<DeadbandTier> deadbandTiers;
};

class StockLuxFilter {
  public:
    void configure(const StockLuxFilterConfig& config);
    std::optional<float> apply(float value);
    void markReported(float value) { mLastReported = value; }
    bool hasReported() const { return mLastReported.has_value(); }
    void reset();

  private:
    StockLuxFilterConfig mConfig{};
    std::deque<double> mLowLux;
    std::deque<double> mMedian;
    std::optional<float> mLastReported;
};

// Schema-driven stock UDALS model evaluator with zero Android runtime dependencies.
class UDalsStockModel {
  public:
    bool load(const std::string& modelPath, const std::string& configPath,
              const std::string& calibrationPath, std::string* error,
              std::string* warning = nullptr);
    bool loadFromStrings(const std::string& modelJson, const std::string& configJson,
                         const std::optional<std::string>& calibrationJson, std::string* error,
                         std::string* warning = nullptr);

    bool isLoaded() const { return mLoaded; }
    const std::string& sensorOwner() const { return mSensorOwner; }
    UdalsGeometry geometry() const { return mGeometry; }
    UdalsRgb maxWhiteRgb() const { return mMaxWhiteRgb; }
    const StockLuxFilterConfig& filterConfig() const { return mFilterConfig; }
    float defaultMsgUnknown1() const { return mDefaultMsgUnknown1; }

    UdalsRgb samplePixels(const uint8_t* rgba, size_t stridePixels, int32_t bufLeft,
                          int32_t bufTop) const;
    UdalsChannels predictBleed(int dbv, bool dc, const UdalsRgb& rgb) const;
    float computeLux(const UdalsChannels& raw, const UdalsChannels& bleed, int dbv) const;

  private:
    enum class InputTransform : uint8_t {
        kAffine,
        kLog,
    };

    enum class Routing : uint8_t {
        kPerChannel,
        kDbvRules,
    };

    struct LuxConfig {
        bool coefScale = false;
        double divisor = 1.0;
        double saturationRaw = 0.0;
        double saturationLux = 0.0;
        int dbvOffsetAbove = 0;
        double dbvOffset = 0.0;
        double lowLuxThreshold = 0.0;
        double lowLuxOffset = 0.0;
    };

    struct Layer {
        size_t inputs = 0;
        size_t outputs = 0;
        std::vector<double> weights;
        std::vector<double> bias;
    };

    struct Network {
        enum class Negative : uint8_t {
            kAllow,
            kClamp,
            kZero,
        };

        std::optional<int> maxDbv;
        std::optional<bool> dc;
        std::vector<Layer> layers;
        std::vector<double> exponent;
        std::vector<double> divisor;
        Negative negative = Negative::kAllow;

        std::vector<double> evaluate(const UdalsChannels& input) const;
    };

    struct Point {
        int dbv = 0;
        UdalsChannels channels{};
    };

    using Curve = std::vector<Point>;

    static UdalsChannels interpolateCurve(const Curve& curve, int dbv);

    bool mLoaded = false;
    std::string mSensorOwner;
    int32_t mGrid = 0;
    int32_t mBlockSize = 0;
    UdalsGeometry mGeometry{};
    std::array<std::vector<double>, 3> mSpatial;
    std::array<double, 256> mGamma{};
    UdalsRgb mMaxWhiteRgb{};
    InputTransform mInputTransform = InputTransform::kAffine;
    UdalsChannels mOffset{};
    UdalsChannels mDivisor{1.0, 1.0, 1.0, 1.0};
    Routing mRouting = Routing::kPerChannel;
    int mDcCurveMaxDbv = 0;
    UdalsChannels mScale{1.0, 1.0, 1.0, 1.0};
    std::vector<double> mCoefficients;
    std::vector<Network> mNetworks;
    Curve mOriginal;
    Curve mCalibrated;
    Curve mOriginalDc;
    Curve mCalibratedDc;
    LuxConfig mLux{};
    StockLuxFilterConfig mFilterConfig{};
    float mDefaultMsgUnknown1 = 2.0f;
};
