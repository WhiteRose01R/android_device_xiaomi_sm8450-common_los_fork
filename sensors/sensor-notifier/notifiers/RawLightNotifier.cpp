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
#include <display/drm/mi_disp.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <vector>

#include "SensorNotifierUtils.h"
#include "SscCalApi.h"

using android::hardware::Return;
using android::hardware::Void;
using android::hardware::sensors::V1_0::Event;

namespace {

constexpr const char kDispFeatureDevice[] = "/dev/mi_display/disp_feature";

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

static void syncDisplayFeatures(int dispFd,
                                const std::shared_ptr<UDalsBleedCompensator>& compensator) {
    if (dispFd < 0 || compensator == nullptr) {
        return;
    }

    disp_feature_req req{};
    req.base.disp_id = MI_DISP_PRIMARY;
    req.feature_id = DISP_FEATURE_DC;
    if (ioctl(dispFd, MI_DISP_IOCTL_GET_FEATURE, &req) == 0) {
        compensator->setDcMode(req.feature_val != FEATURE_OFF);
    }
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

void RawLightNotifier::syncCompensatorDisplayState(int dispFd) {
    if (mCompensator == nullptr) {
        return;
    }
    resolveBacklightScale();

    bool synced = false;
    if (dispFd >= 0) {
        disp_brightness_req req{};
        req.base.disp_id = MI_DISP_PRIMARY;
        if (ioctl(dispFd, MI_DISP_IOCTL_GET_BRIGHTNESS, &req) == 0) {
            updateBacklight(req.brightness);
            synced = true;
        }
    }

    if (!synced) {
        uint32_t bkl = 0;
        if (readPanelSysfsUint("brightness", &bkl)) {
            updateBacklight(bkl);
        }
    }

    syncDisplayFeatures(dispFd, mCompensator);
}

RawLightNotifier::RawLightNotifier(sp<ISensorManager> manager)
    : SensorNotifier(manager),
      mEnabled(false),
      mBacklightScaleResolved(false),
      mDeviceNormalMaxBacklight(0u) {
    auto compensator = std::make_shared<UDalsBleedCompensator>();
    UdalsGeometry geo;

    static constexpr const char* kConfigPaths[] = {
            "/odm/etc/sensors/config/lightSensorConfig.json",
            "/vendor/etc/sensors/config/lightSensorConfig.json",
    };

    int wakeFds[2] = {-1, -1};
    if (pipe2(wakeFds, O_CLOEXEC | O_NONBLOCK) == 0) {
        mWakeReadFd.reset(wakeFds[0]);
        mWakeWriteFd.reset(wakeFds[1]);
    } else {
        PLOG(WARNING) << "Failed to create RawLightNotifier wake pipe";
    }

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
    if (mWakeWriteFd.get() != -1) {
        const uint8_t wakeByte = 1;
        ssize_t ignored =
                TEMP_FAILURE_RETRY(write(mWakeWriteFd.get(), &wakeByte, sizeof(wakeByte)));
        (void)ignored;
    }
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

    android::base::unique_fd disp_fd_ = android::base::unique_fd(open(kDispFeatureDevice, O_RDWR));
    if (disp_fd_.get() == -1) {
        PLOG(ERROR) << "failed to open " << kDispFeatureDevice;
        mActive = false;
        return;
    }

    std::vector<disp_event_type> notifyEvents = {MI_DISP_EVENT_POWER, MI_DISP_EVENT_51_BRIGHTNESS};
    if (mCompensator != nullptr) {
        notifyEvents.push_back(MI_DISP_EVENT_DC);
    }

    for (const disp_event_type& event : notifyEvents) {
        disp_event_req req{};
        req.base.flag = 0;
        req.base.disp_id = MI_DISP_PRIMARY;
        req.type = event;
        if (ioctl(disp_fd_.get(), MI_DISP_IOCTL_REGISTER_EVENT, &req) < 0) {
            PLOG(ERROR) << "failed to register display event " << event;
        }
    }

    bool displayOn = false;
    bool powerEventSeen = false;

    auto enterDisplayOn = [&](const uint32_t* initialRawBkl) {
        if (displayOn) {
            return;
        }
        if (mCompensator != nullptr) {
            mCompensator->reset();
            if (initialRawBkl != nullptr) {
                resolveBacklightScale();
                updateBacklight(*initialRawBkl);
                syncDisplayFeatures(disp_fd_.get(), mCompensator);
            } else {
                syncCompensatorDisplayState(disp_fd_.get());
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

    disp_doze_brightness_req dozeReq{};
    dozeReq.base.disp_id = MI_DISP_PRIMARY;
    bool isDoze = (ioctl(disp_fd_.get(), MI_DISP_IOCTL_GET_DOZE_BRIGHTNESS, &dozeReq) == 0 &&
                   dozeReq.doze_brightness != DOZE_TO_NORMAL);
    if (isDoze) {
        powerEventSeen = true;
    } else {
        disp_brightness_req blReq{};
        blReq.base.disp_id = MI_DISP_PRIMARY;
        uint32_t startupBl = 0;
        if (ioctl(disp_fd_.get(), MI_DISP_IOCTL_GET_BRIGHTNESS, &blReq) == 0) {
            startupBl = blReq.brightness;
        } else {
            readPanelSysfsUint("brightness", &startupBl);
        }
        if (startupBl > 0) {
            enterDisplayOn(&startupBl);
        }
    }

    std::vector<struct pollfd> pollfds;
    pollfds.push_back({.fd = disp_fd_.get(), .events = POLLIN, .revents = 0});
    int wakeIdx = -1;
    if (mWakeReadFd.get() != -1) {
        wakeIdx = static_cast<int>(pollfds.size());
        pollfds.push_back({.fd = mWakeReadFd.get(), .events = POLLIN, .revents = 0});
    }
    int pollTimeoutMs = (wakeIdx >= 0) ? -1 : 500;

    while (mActive) {
        for (auto& pfd : pollfds) {
            pfd.revents = 0;
        }
        int rc = poll(pollfds.data(), pollfds.size(), pollTimeoutMs);
        if (rc == 0) {
            continue;
        }
        if (rc < 0) {
            if (errno != EINTR) {
                PLOG(ERROR) << "failed to poll " << kDispFeatureDevice;
            }
            continue;
        }
        if (wakeIdx >= 0 && (pollfds[static_cast<size_t>(wakeIdx)].revents & POLLIN)) {
            uint8_t buf[16];
            while (read(mWakeReadFd.get(), buf, sizeof(buf)) > 0) {
            }
            if (!mActive) {
                break;
            }
        }
        if ((pollfds[0].revents & POLLIN) == 0) {
            continue;
        }

        std::shared_ptr<disp_event_resp> response = parseDispEvent(disp_fd_.get());
        if (response == nullptr) {
            continue;
        }

        if (response->base.disp_id != MI_DISP_PRIMARY) {
            continue;
        }

        size_t payloadLength = response->base.length > sizeof(response->base)
                                       ? response->base.length - sizeof(response->base)
                                       : 0;

        switch (response->base.type) {
            case MI_DISP_EVENT_POWER: {
                if (payloadLength < sizeof(int32_t)) {
                    continue;
                }
                powerEventSeen = true;
                int32_t powerVal = 0;
                std::memcpy(&powerVal, response->data, sizeof(powerVal));
                if (powerVal == MI_DISP_POWER_ON) {
                    enterDisplayOn(nullptr);
                } else {
                    enterDisplayOff();
                }
                break;
            }
            case MI_DISP_EVENT_51_BRIGHTNESS: {
                uint32_t bklRaw = 0;
                if (payloadLength >= sizeof(uint32_t)) {
                    std::memcpy(&bklRaw, response->data, sizeof(bklRaw));
                } else if (payloadLength >= sizeof(uint16_t)) {
                    uint16_t bkl16 = 0;
                    std::memcpy(&bkl16, response->data, sizeof(bkl16));
                    bklRaw = bkl16;
                } else {
                    continue;
                }
                if (!displayOn) {
                    // Recover if POWER_ON fired before event registration at boot;
                    // the 5.10 driver suppresses MI_DISP_EVENT_51_BRIGHTNESS during AOD.
                    if (!powerEventSeen && bklRaw > 0) {
                        enterDisplayOn(&bklRaw);
                    }
                    break;
                }
                updateBacklight(bklRaw);
                break;
            }
            case MI_DISP_EVENT_DC: {
                if (payloadLength < sizeof(int32_t)) {
                    continue;
                }
                int32_t dcVal = 0;
                std::memcpy(&dcVal, response->data, sizeof(dcVal));
                if (mCompensator != nullptr) {
                    mCompensator->setDcMode(dcVal != 0);
                }
                break;
            }
            default:
                LOG(ERROR) << "got unknown event: " << response->base.type;
                continue;
        }
    }
}
