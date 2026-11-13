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
#include <fcntl.h>
#include <poll.h>
#include <cerrno>

#include "SensorNotifierUtils.h"
#include "SscCalApi.h"

using android::hardware::Return;
using android::hardware::Void;
using android::hardware::sensors::V1_0::Event;

namespace {

constexpr const char kPanelPowerStateNode[] =
        "/sys/devices/platform/soc/soc:qcom,dsi-display-primary/panel_power_state";
constexpr int kPanelPowerOn = 0;

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

    android::base::unique_fd powerFd(open(kPanelPowerStateNode, O_RDONLY));
    if (powerFd.get() == -1) {
        PLOG(ERROR) << "failed to open " << kPanelPowerStateNode;
        mActive = false;
        return;
    }

    struct pollfd powerPoll = {
            .fd = powerFd.get(),
            .events = POLLERR | POLLPRI,
            .revents = POLLPRI,
    };

    while (mActive) {
        if (powerPoll.revents & (POLLERR | POLLPRI)) {
            int powerVal = 0;
            if (readInt(powerPoll.fd, &powerVal)) {
                if (powerVal == kPanelPowerOn) {
                    if (!mEnabled) {
                        Result res = mQueue->enableSensor(mSensorHandle, 20000 /* sample period */,
                                                          0 /* latency */);
                        if (res != Result::OK) {
                            LOG(ERROR) << "failed to enable sensor";
                        } else {
                            mEnabled = true;
                        }
                    }
                } else {
                    if (mEnabled) {
                        Result res = mQueue->disableSensor(mSensorHandle);
                        if (res != Result::OK) {
                            LOG(ERROR) << "failed to disable sensor";
                        } else {
                            mEnabled = false;
                        }
                    }
                }
            }
        }

        powerPoll.revents = 0;
        int rc = poll(&powerPoll, 1, 500);
        if (rc < 0 && errno != EINTR) {
            PLOG(ERROR) << "failed to poll " << kPanelPowerStateNode;
        }
    }
}
