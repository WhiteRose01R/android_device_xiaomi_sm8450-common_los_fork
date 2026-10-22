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
    explicit RawLightSensorCallback(std::shared_ptr<UDalsBleedCompensator> compensator)
        : mCompensator(std::move(compensator)) {}

    Return<void> onEvent(const Event& e) override {
        float rawLux = e.u.vec4.y;
        float compensatedLux = rawLux;

        if (mCompensator != nullptr) {
            if (!mCompensator->compensate(rawLux, &compensatedLux)) {
                return Void();
            }
        }

        _oem_msg msg{};
        msg.notifyType = REPORT_VALUE;
        msg.value = compensatedLux;
        msg.notifyTypeFloat = static_cast<float>(msg.notifyType);
        msg.unknown1 = 2.0f;
        msg.unknown2 = 5.0f;
        msg.sensorType = kSensorTypeAmbientlightRaw;

        SscCalApiWrapper::getInstance().processMsg(&msg);

        LOG(VERBOSE) << "UDALS event: raw=" << rawLux << " compensated=" << compensatedLux
                     << " bleed=" << (rawLux - compensatedLux);

        return Void();
    }

  private:
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
};

// Check panel0 first (primary display); fallback to panel1 only if panel0 sysfs node is absent.
static uint32_t readPanelBacklight() {
    static constexpr const char* kBacklightPaths[] = {
            "/sys/class/backlight/panel0-backlight/brightness",
            "/sys/class/backlight/panel1-backlight/brightness",
    };
    for (const char* path : kBacklightPaths) {
        std::string content;
        if (android::base::ReadFileToString(path, &content)) {
            uint32_t bkl = 0;
            if (android::base::ParseUint(android::base::Trim(content), &bkl)) {
                return bkl;
            }
        }
    }
    return 0;
}

static uint32_t determineNormalMaxBacklight() {
    return android::base::GetUintProperty<uint32_t>(
            "ro.vendor.sensors.notifier.udals.normal_max_brightness", 2047u);
}

static bool parseRoiProperty(const std::string& prop, UdalsRoi* outRoi) {
    if (prop.empty()) {
        return false;
    }
    const std::vector<std::string> parts = android::base::Split(prop, ",");
    if (parts.size() != 4) {
        return false;
    }
    int l, t, r, b;
    if (!android::base::ParseInt(android::base::Trim(parts[0]), &l) ||
        !android::base::ParseInt(android::base::Trim(parts[1]), &t) ||
        !android::base::ParseInt(android::base::Trim(parts[2]), &r) ||
        !android::base::ParseInt(android::base::Trim(parts[3]), &b)) {
        return false;
    }
    if (l < 0 || t < 0 || r <= l || b <= t) {
        return false;
    }
    int width = r - l;
    int height = b - t;
    if (width < 4 || width > 512 || height < 4 || height > 512) {
        return false;
    }
    outRoi->left = l;
    outRoi->top = t;
    outRoi->right = r;
    outRoi->bottom = b;
    return true;
}

}  // namespace

uint32_t RawLightNotifier::normalizeBacklight(uint32_t raw) const {
    if (mNormalMaxBacklight == 0 || mNormalMaxBacklight == 2047) {
        return raw;
    }
    return static_cast<uint32_t>((static_cast<uint64_t>(raw) * 2047u + (mNormalMaxBacklight / 2)) /
                                 mNormalMaxBacklight);
}

RawLightNotifier::RawLightNotifier(sp<ISensorManager> manager)
    : SensorNotifier(manager), mEnabled(false), mNormalMaxBacklight(determineNormalMaxBacklight()) {
    std::string roiProp = android::base::GetProperty("ro.vendor.sensors.notifier.udals.roi", "");
    UdalsRoi roi;
    if (parseRoiProperty(roiProp, &roi)) {
        mCompensator = std::make_shared<UDalsBleedCompensator>();
        mCwbClient = std::make_unique<CwbClient>(roi, mCompensator);
        uint32_t initBkl = readPanelBacklight();
        if (initBkl > 0) {
            mCompensator->setBacklight(normalizeBacklight(initBkl));
        }
    } else {
        LOG(DEBUG) << "No UDALS CWB ROI property configured; running without bleed compensation";
    }

    initializeSensorQueue("xiaomi.sensor.ambientlight.factory", false,
                          new RawLightSensorCallback(mCompensator));
}

RawLightNotifier::~RawLightNotifier() {
    deactivate();
    if (mCwbClient != nullptr) {
        mCwbClient->stop();
    }
    if (mEnabled && mQueue != nullptr) {
        mQueue->disableSensor(mSensorHandle);
        mEnabled = false;
    }
}

void RawLightNotifier::notify() {
    if (mQueue == nullptr) {
        LOG(DEBUG) << "Factory raw light sensor queue unavailable, skipping RawLightNotifier";
        mActive = false;
        return;
    }

    Result res;

    android::base::unique_fd disp_fd_ = android::base::unique_fd(open(kDispFeatureDevice, O_RDWR));
    if (disp_fd_.get() == -1) {
        PLOG(ERROR) << "failed to open " << kDispFeatureDevice;
        mActive = false;
        return;
    }

    const std::vector<disp_event_type> notifyEvents = {MI_DISP_EVENT_POWER, MI_DISP_EVENT_FPS,
                                                       MI_DISP_EVENT_51_BRIGHTNESS,
                                                       MI_DISP_EVENT_HBM, MI_DISP_EVENT_DC};

    for (const disp_event_type& event : notifyEvents) {
        disp_event_req req;
        req.base.flag = 0;
        req.base.disp_id = MI_DISP_PRIMARY;
        req.type = event;
        if (ioctl(disp_fd_.get(), MI_DISP_IOCTL_REGISTER_EVENT, &req) < 0) {
            PLOG(ERROR) << "failed to register display event " << event;
        }
    }

    res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */, 0 /* latency */);
    if (res != Result::OK) {
        LOG(ERROR) << "failed to enable sensor";
    } else {
        mEnabled = true;
        if (mCwbClient != nullptr) {
            mCwbClient->start();
        }
    }

    struct pollfd dispEventPoll = {
            .fd = disp_fd_.get(),
            .events = POLLIN,
    };

    _oem_msg msg{};
    notify_t notifyType;
    float value;

    while (mActive) {
        int rc = poll(&dispEventPoll, 1, 500);
        if (rc == 0) {
            continue;
        }
        if (rc < 0) {
            if (errno != EINTR) {
                PLOG(ERROR) << "failed to poll " << kDispFeatureDevice;
            }
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
                int32_t powerVal = 0;
                std::memcpy(&powerVal, response->data, sizeof(powerVal));
                notifyType = POWER_STATE;
                value = static_cast<float>(powerVal);
                switch (powerVal) {
                    case MI_DISP_POWER_ON:
                        if (mCompensator != nullptr) {
                            mCompensator->reset();
                            uint32_t bkl = readPanelBacklight();
                            if (bkl > 0) {
                                mCompensator->setBacklight(normalizeBacklight(bkl));
                            }
                        }
                        if (!mEnabled) {
                            res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */,
                                                       0 /* latency */);
                            if (res != Result::OK) {
                                LOG(ERROR) << "failed to enable sensor";
                            } else {
                                mEnabled = true;
                                if (mCwbClient != nullptr) {
                                    mCwbClient->start();
                                }
                            }
                        }
                        break;
                    default:
                        if (mEnabled) {
                            res = mQueue->disableSensor(mSensorHandle);
                            if (res != Result::OK) {
                                LOG(ERROR) << "failed to disable sensor";
                            } else {
                                mEnabled = false;
                            }
                        }
                        if (mCwbClient != nullptr) {
                            mCwbClient->stop();
                        }
                        if (mCompensator != nullptr) {
                            mCompensator->reset();
                        }
                        break;
                }
                break;
            }
            case MI_DISP_EVENT_FPS: {
                if (payloadLength < sizeof(int32_t)) {
                    continue;
                }
                int32_t fpsVal = 0;
                std::memcpy(&fpsVal, response->data, sizeof(fpsVal));
                notifyType = DISPLAY_FREQUENCY;
                value = static_cast<float>(fpsVal);
                break;
            }
            case MI_DISP_EVENT_51_BRIGHTNESS: {
                if (payloadLength < sizeof(uint16_t)) {
                    continue;
                }
                uint16_t bklRaw = 0;
                std::memcpy(&bklRaw, response->data, sizeof(bklRaw));
                notifyType = BRIGHTNESS;
                value = static_cast<float>(bklRaw);
                if (mCompensator != nullptr) {
                    mCompensator->setBacklight(normalizeBacklight(static_cast<uint32_t>(bklRaw)));
                }
                break;
            }
            case MI_DISP_EVENT_HBM: {
                if (payloadLength < sizeof(int32_t)) {
                    continue;
                }
                int32_t hbmVal = 0;
                std::memcpy(&hbmVal, response->data, sizeof(hbmVal));
                notifyType = BRIGHTNESS;
                value = hbmVal ? -1.0f : -2.0f;
                if (mCompensator != nullptr) {
                    mCompensator->setHbmMode(hbmVal != 0);
                }
                break;
            }
            case MI_DISP_EVENT_DC: {
                if (payloadLength < sizeof(int32_t)) {
                    continue;
                }
                int32_t dcVal = 0;
                std::memcpy(&dcVal, response->data, sizeof(dcVal));
                notifyType = DC_STATE;
                value = static_cast<float>(dcVal);
                if (mCompensator != nullptr) {
                    mCompensator->setDcMode(dcVal != 0);
                }
                break;
            }
            default:
                LOG(ERROR) << "got unknown event: " << response->base.type;
                continue;
        }

        msg = {};
        msg.sensorType = kSensorTypeAmbientlightRaw;
        msg.notifyType = notifyType;
        msg.notifyTypeFloat = static_cast<float>(notifyType);
        msg.value = value;
        msg.unknown1 = 1.0f;
        msg.unknown2 = 5.0f;

        SscCalApiWrapper::getInstance().processMsg(&msg);
    }
}
