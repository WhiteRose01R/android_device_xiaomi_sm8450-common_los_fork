/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "RawLightNotifier"

#include "RawLightNotifier.h"

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/unique_fd.h>
#include <display/drm/mi_disp.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

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
    explicit RawLightSensorCallback(float msgUnknown1) : mMsgUnknown1(msgUnknown1) {}

    Return<void> onEvent(const Event& e) override {
        _oem_msg msg{};
        msg.notifyType = REPORT_VALUE;
        msg.value = e.u.vec4.y;
        msg.notifyTypeFloat = static_cast<float>(msg.notifyType);
        msg.unknown1 = mMsgUnknown1;
        msg.unknown2 = 5.0f;
        msg.sensorType = kSensorTypeAmbientlightRaw;

        SscCalApiWrapper::getInstance().processMsg(&msg);

        return Void();
    }

  private:
    float mMsgUnknown1;
};

}  // namespace

RawLightNotifier::RawLightNotifier(sp<ISensorManager> manager)
    : SensorNotifier(manager), mEnabled(false) {
    float msgUnknown1 = static_cast<float>(android::base::GetUintProperty<uint32_t>(
            "ro.vendor.sensors.notifier.raw_light.unknown1", 2u));
    initializeSensorQueue("xiaomi.sensor.ambientlight.factory", false,
                          new RawLightSensorCallback(msgUnknown1));
}

RawLightNotifier::~RawLightNotifier() {
    mActive = false;
    deactivate();
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

    disp_event_req req{};
    req.base.flag = 0;
    req.base.disp_id = MI_DISP_PRIMARY;
    req.type = MI_DISP_EVENT_POWER;
    if (ioctl(disp_fd_.get(), MI_DISP_IOCTL_REGISTER_EVENT, &req) < 0) {
        PLOG(ERROR) << "failed to register display power event";
    }

    Result res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */, 0 /* latency */);
    if (res != Result::OK) {
        LOG(ERROR) << "failed to enable sensor";
    } else {
        mEnabled = true;
    }

    struct pollfd dispEventPoll = {
            .fd = disp_fd_.get(),
            .events = POLLIN,
            .revents = 0,
    };

    while (mActive) {
        dispEventPoll.revents = 0;
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
        if ((dispEventPoll.revents & POLLIN) == 0) {
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

        if (response->base.type != MI_DISP_EVENT_POWER) {
            LOG(ERROR) << "unexpected display event: " << response->base.type;
            continue;
        }

        if (payloadLength < sizeof(int32_t)) {
            continue;
        }

        int32_t powerVal = 0;
        std::memcpy(&powerVal, response->data, sizeof(powerVal));
        switch (powerVal) {
            case MI_DISP_POWER_ON:
                if (!mEnabled) {
                    res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */,
                                               0 /* latency */);
                    if (res != Result::OK) {
                        LOG(ERROR) << "failed to enable sensor";
                    } else {
                        mEnabled = true;
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
                break;
        }
    }
}
