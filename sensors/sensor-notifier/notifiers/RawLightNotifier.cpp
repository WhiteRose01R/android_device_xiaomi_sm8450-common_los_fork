/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "RawLightNotifier"

#include "RawLightNotifier.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parseint.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <android-base/unique_fd.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <vector>

#include "SensorNotifierUtils.h"
#include "SscCalApi.h"

using android::hardware::Return;
using android::hardware::Void;
using android::hardware::sensors::V1_0::Event;

namespace {

constexpr const char kPanelPowerStateNode[] =
        "/sys/devices/platform/soc/soc:qcom,dsi-display-primary/panel_power_state";
constexpr const char kDbvNode[] = "/sys/devices/platform/soc/soc:qcom,dsi-display-primary/dbv";
constexpr int kPanelPowerOn = 0;
constexpr int kPanelPowerOff = 5;

// Xiaomi SSC vendor sensor type for raw ambient light
constexpr uint32_t kSensorTypeAmbientlightRaw = 33171111;

class RawLightSensorCallback : public IEventQueueCallback {
  public:
    RawLightSensorCallback(std::shared_ptr<UDalsBleedCompensator> compensator,
                           std::weak_ptr<CwbClient> cwbClient, float msgUnknown1)
        : mCompensator(std::move(compensator)),
          mCwbClient(std::move(cwbClient)),
          mMsgUnknown1(msgUnknown1) {}

    Return<void> onEvent(const Event& e) override {
        float rawLux = e.u.vec4.y;
        float compensatedLux = rawLux;

        if (auto cwb = mCwbClient.lock()) {
            cwb->onRawLuxSample(rawLux);
        }

        if (mCompensator != nullptr) {
            if (!mCompensator->compensate(rawLux, &compensatedLux)) {
                return Void();
            }
        }

        _oem_msg msg{};
        msg.notifyType = REPORT_VALUE;
        msg.value = compensatedLux;
        msg.notifyTypeFloat = static_cast<float>(msg.notifyType);
        msg.unknown1 = mMsgUnknown1;
        msg.unknown2 = 5.0f;
        msg.sensorType = kSensorTypeAmbientlightRaw;

        SscCalApiWrapper::getInstance().processMsg(&msg);

        LOG(VERBOSE) << "UDALS event: raw=" << rawLux << " compensated=" << compensatedLux
                     << " bleed=" << (rawLux - compensatedLux);

        return Void();
    }

  private:
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::weak_ptr<CwbClient> mCwbClient;
    float mMsgUnknown1;
};

// Check panel0 first (primary display); fallback to panel1 only if panel0 sysfs node is absent.
static bool readPanelSysfsUint(const char* attr, uint32_t* out) {
    static std::atomic<bool> loggedReadError{false};
    for (const char* panel : {"panel0-backlight", "panel1-backlight"}) {
        std::string content;
        std::string path = std::string("/sys/class/backlight/") + panel + "/" + attr;
        if (android::base::ReadFileToString(path, &content)) {
            uint32_t val = 0;
            if (android::base::ParseUint(android::base::Trim(content), &val)) {
                *out = val;
                return true;
            }
        } else if (errno != ENOENT && errno != ENODEV && !loggedReadError.exchange(true)) {
            PLOG(WARNING) << "Failed to read " << path;
        }
    }
    return false;
}

}  // namespace

void RawLightNotifier::resolveBacklightScale() {
    if (mBacklightScaleResolved) {
        return;
    }

    uint32_t propMax = android::base::GetUintProperty<uint32_t>(
            "ro.vendor.sensors.notifier.udals.normal_max_brightness", 0u);
    if (propMax > 0) {
        mDeviceNormalMaxBacklight = propMax;
        mBacklightScaleResolved = true;
        return;
    }

    uint32_t sysfsMax = 0;
    if (readPanelSysfsUint("max_brightness", &sysfsMax) && sysfsMax > 1) {
        mDeviceNormalMaxBacklight = sysfsMax / 2;
        mBacklightScaleResolved = true;
        return;
    }

    // Leave unresolved so the next syncCompensatorDisplayState() retries.
    // normalizeBacklight() already handles the 0 case as identity.
    mDeviceNormalMaxBacklight = 0;
}

uint32_t RawLightNotifier::normalizeBacklight(uint32_t raw) const {
    uint32_t targetMax = (mCompensator != nullptr) ? mCompensator->getNormalMaxBacklight() : 2047u;
    if (mDeviceNormalMaxBacklight == 0 || mDeviceNormalMaxBacklight == targetMax) {
        return raw;
    }
    return static_cast<uint32_t>(
            (static_cast<uint64_t>(raw) * targetMax + (mDeviceNormalMaxBacklight / 2)) /
            mDeviceNormalMaxBacklight);
}

void RawLightNotifier::applyNormalizedBacklight(uint32_t normalized) {
    if (mCompensator == nullptr) {
        return;
    }
    uint32_t prevNormalized = mCompensator->getBacklight();
    mCompensator->setBacklight(normalized);
    if (prevNormalized == 0 && normalized > 0 && mCwbClient != nullptr) {
        mCwbClient->notifyBacklightChanged();
    }
}

void RawLightNotifier::updateBacklight(uint32_t rawBacklight) {
    if (mCompensator == nullptr) {
        return;
    }
    applyNormalizedBacklight(normalizeBacklight(rawBacklight));
}

bool RawLightNotifier::setSensorEnabled(bool enable) {
    if (enable == mEnabled) {
        return true;
    }
    Result res =
            enable ? mQueue->enableSensor(mSensorHandle, 20000 /* sample period */, 0 /* latency */)
                   : mQueue->disableSensor(mSensorHandle);
    if (res != Result::OK) {
        LOG(ERROR) << "failed to " << (enable ? "enable" : "disable") << " sensor";
        return false;
    }
    mEnabled = enable;
    return true;
}

void RawLightNotifier::syncCompensatorDisplayState(int dbvFd) {
    if (mCompensator == nullptr) {
        return;
    }
    resolveBacklightScale();

    bool synced = false;
    if (dbvFd >= 0) {
        int dbvVal = 0;
        if (readInt(dbvFd, &dbvVal) && dbvVal >= 0) {
            updateBacklight(static_cast<uint32_t>(dbvVal));
            synced = true;
        }
    }

    if (!synced) {
        uint32_t bkl = 0;
        if (readPanelSysfsUint("brightness", &bkl)) {
            updateBacklight(bkl);
        }
    }
}

RawLightNotifier::RawLightNotifier(sp<ISensorManager> manager)
    : SensorNotifier(manager),
      mEnabled(false),
      mBacklightScaleResolved(false),
      mDeviceNormalMaxBacklight(0u) {
    auto compensator = std::make_shared<UDalsBleedCompensator>();
    UdalsGeometry geo;

    static constexpr const char* kConfigPaths[] = {
            "/vendor/etc/sensors/config/lightSensorConfig.json",
            "/odm/etc/sensors/config/lightSensorConfig.json",
    };

    bool loaded = false;
    for (const char* path : kConfigPaths) {
        if (compensator->loadConfig(path, &geo)) {
            loaded = true;
            break;
        }
    }

    if (loaded) {
        mCompensator = compensator;
        resolveBacklightScale();
        mCwbClient = std::make_shared<CwbClient>(geo, mCompensator);
    } else {
        mBacklightScaleResolved = true;
        LOG(INFO) << "No UDALS CWB configuration available; running uncompensated";
    }

    float msgUnknown1 = static_cast<float>(android::base::GetUintProperty<uint32_t>(
            "ro.vendor.sensors.notifier.raw_light.unknown1", 2u));
    initializeSensorQueue("xiaomi.sensor.ambientlight.factory", false,
                          new RawLightSensorCallback(mCompensator, mCwbClient, msgUnknown1));
}

RawLightNotifier::~RawLightNotifier() {
    mActive = false;
    deactivate();
    if (mCwbClient != nullptr) {
        mCwbClient->stop();
    }
    mEnabled = false;
}

void RawLightNotifier::notify() {
    if (mQueue == nullptr) {
        LOG(DEBUG) << "Factory raw light sensor queue unavailable, skipping RawLightNotifier";
        mActive = false;
        return;
    }

    android::base::unique_fd powerFd(open(kPanelPowerStateNode, O_RDONLY));
    if (powerFd.get() == -1) {
        PLOG(ERROR) << "failed to open " << kPanelPowerStateNode;
        mActive = false;
        return;
    }

    android::base::unique_fd dbvFd(open(kDbvNode, O_RDONLY));
    if (dbvFd.get() == -1) {
        PLOG(WARNING) << "failed to open " << kDbvNode;
    }

    std::vector<struct pollfd> pollfds;
    pollfds.push_back({.fd = powerFd.get(), .events = POLLERR | POLLPRI, .revents = POLLPRI});
    if (dbvFd.get() != -1) {
        pollfds.push_back({.fd = dbvFd.get(), .events = POLLERR | POLLPRI, .revents = POLLPRI});
    }

    bool displayOn = false;
    bool powerEventSeen = false;
    bool isInitialRead = true;
    int lastPowerVal = -1;

    auto enterDisplayOn = [&](const uint32_t* initialRawDbv) {
        if (displayOn) {
            return;
        }
        if (mCompensator != nullptr) {
            mCompensator->reset();
            if (initialRawDbv != nullptr) {
                resolveBacklightScale();
                updateBacklight(*initialRawDbv);
            } else {
                syncCompensatorDisplayState(dbvFd.get());
            }
        }
        if (!setSensorEnabled(true)) {
            return;
        }
        displayOn = true;
        if (mActive && mCwbClient != nullptr) {
            mCwbClient->start();
        }
    };

    auto enterDisplayOff = [&]() {
        if (!displayOn) {
            return;
        }
        displayOn = false;
        setSensorEnabled(false);
        if (mCwbClient != nullptr) {
            mCwbClient->stop();
        }
        if (mCompensator != nullptr) {
            mCompensator->reset();
        }
    };

    while (mActive) {
        if (pollfds[0].revents & (POLLERR | POLLPRI)) {
            int powerVal = 0;
            if (readInt(pollfds[0].fd, &powerVal)) {
                if (!isInitialRead) {
                    powerEventSeen = true;
                }
                lastPowerVal = powerVal;
                if (powerVal == kPanelPowerOn) {
                    enterDisplayOn(nullptr);
                } else {
                    enterDisplayOff();
                }
            }
        }

        if (pollfds.size() > 1 && (pollfds[1].revents & (POLLERR | POLLPRI))) {
            int dbvVal = 0;
            if (readInt(pollfds[1].fd, &dbvVal) && dbvVal >= 0) {
                if (!displayOn) {
                    // Fallback for unpatched 6.6 kernels that leave panel_power_state at
                    // OFF (5) after continuous splash until first lock/unlock; AOD uses LP1/LP2.
                    if (!powerEventSeen && lastPowerVal == kPanelPowerOff && dbvVal > 0) {
                        uint32_t sysfsBl = 0;
                        bool sysfsOk = readPanelSysfsUint("brightness", &sysfsBl);
                        if (!sysfsOk || sysfsBl > 0) {
                            uint32_t rawDbv = static_cast<uint32_t>(dbvVal);
                            enterDisplayOn(&rawDbv);
                        }
                    }
                } else {
                    updateBacklight(static_cast<uint32_t>(dbvVal));
                }
            }
        }

        isInitialRead = false;

        for (auto& pfd : pollfds) {
            pfd.revents = 0;
        }

        int rc = poll(pollfds.data(), pollfds.size(), 500);
        if (rc < 0 && errno != EINTR) {
            PLOG(ERROR) << "failed to poll display nodes";
        }
    }
}
