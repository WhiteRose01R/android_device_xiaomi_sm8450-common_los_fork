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
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <limits>
#include <optional>
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

constexpr auto kInitialEnableRetryInterval = std::chrono::milliseconds(500);
constexpr auto kMaxEnableRetryInterval = std::chrono::milliseconds(10000);

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

bool readPanelSysfsUint(const char* attr, uint32_t* out) {
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

bool RawLightNotifier::setSensorEnabled(bool enable) {
    if (enable == mEnabled) {
        return true;
    }
    auto res =
            enable ? mQueue->enableSensor(mSensorHandle, 20000 /* sample period */, 0 /* latency */)
                   : mQueue->disableSensor(mSensorHandle);
    if (!res.isOk() || static_cast<Result>(res) != Result::OK) {
        LOG(ERROR) << "failed to " << (enable ? "enable" : "disable") << " sensor";
        return false;
    }
    mEnabled = enable;
    return true;
}

RawLightNotifier::RawLightNotifier(sp<ISensorManager> manager) : SensorNotifier(manager) {
    int wakeFds[2] = {-1, -1};
    if (pipe2(wakeFds, O_CLOEXEC | O_NONBLOCK) == 0) {
        mWakeReadFd.reset(wakeFds[0]);
        mWakeWriteFd.reset(wakeFds[1]);
    } else {
        PLOG(WARNING) << "Failed to create RawLightNotifier wake pipe";
    }

    float msgUnknown1 = static_cast<float>(android::base::GetUintProperty<uint32_t>(
            "ro.vendor.sensors.notifier.raw_light.unknown1", 2u));
    initializeSensorQueue("xiaomi.sensor.ambientlight.factory", false,
                          new RawLightSensorCallback(msgUnknown1));
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
    int dbvIdx = -1;
    if (dbvFd.get() != -1) {
        dbvIdx = static_cast<int>(pollfds.size());
        pollfds.push_back({.fd = dbvFd.get(), .events = POLLERR | POLLPRI, .revents = POLLPRI});
    }
    int wakeIdx = -1;
    if (mWakeReadFd.get() != -1) {
        wakeIdx = static_cast<int>(pollfds.size());
        pollfds.push_back({.fd = mWakeReadFd.get(), .events = POLLIN, .revents = 0});
    }
    int pollTimeoutMs = (wakeIdx >= 0) ? -1 : 500;

    bool displayOn = false;
    bool wantDisplayOn = false;
    bool powerEventSeen = false;
    bool isInitialRead = true;
    int lastPowerVal = -1;
    std::optional<std::chrono::steady_clock::time_point> nextEnableRetry;
    auto enableRetryDelay = kInitialEnableRetryInterval;

    auto enterDisplayOn = [&]() {
        wantDisplayOn = true;
        if (displayOn) {
            return;
        }
        if (!setSensorEnabled(true)) {
            nextEnableRetry = std::chrono::steady_clock::now() + enableRetryDelay;
            enableRetryDelay = std::min(enableRetryDelay * 2, kMaxEnableRetryInterval);
            return;
        }
        nextEnableRetry.reset();
        enableRetryDelay = kInitialEnableRetryInterval;
        displayOn = true;
    };

    auto enterDisplayOff = [&](bool disableSensor) {
        wantDisplayOn = false;
        nextEnableRetry.reset();
        enableRetryDelay = kInitialEnableRetryInterval;
        if (!displayOn) {
            return;
        }
        displayOn = false;
        if (disableSensor) {
            setSensorEnabled(false);
        }
    };

    auto computePollTimeoutMs = [&]() -> int {
        if (!wantDisplayOn || displayOn || !nextEnableRetry) {
            return pollTimeoutMs;
        }
        auto now = std::chrono::steady_clock::now();
        if (now >= *nextEnableRetry) {
            return 0;
        }
        auto remainingMs = static_cast<int64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(*nextEnableRetry - now)
                        .count());
        int clamped = static_cast<int>(
                std::clamp<int64_t>(remainingMs, 1, std::numeric_limits<int>::max()));
        return (pollTimeoutMs < 0) ? clamped : std::min(pollTimeoutMs, clamped);
    };

    while (mActive) {
        if (wakeIdx >= 0 && (pollfds[static_cast<size_t>(wakeIdx)].revents & POLLIN)) {
            uint8_t buf[16];
            while (read(mWakeReadFd.get(), buf, sizeof(buf)) > 0) {
            }
            if (!mActive) {
                break;
            }
        }

        if (pollfds[0].revents & (POLLERR | POLLPRI)) {
            int powerVal = 0;
            if (readInt(pollfds[0].fd, &powerVal)) {
                if (!isInitialRead) {
                    powerEventSeen = true;
                }
                lastPowerVal = powerVal;
                if (powerVal == kPanelPowerOn) {
                    enableRetryDelay = kInitialEnableRetryInterval;
                    enterDisplayOn();
                } else {
                    enterDisplayOff(/*disableSensor=*/true);
                }
            }
        }

        if (dbvIdx >= 0 && (pollfds[static_cast<size_t>(dbvIdx)].revents & (POLLERR | POLLPRI))) {
            int dbvVal = 0;
            if (readInt(pollfds[static_cast<size_t>(dbvIdx)].fd, &dbvVal) && dbvVal >= 0) {
                if (!displayOn) {
                    // Fallback for unpatched 6.6 kernels that leave panel_power_state at
                    // OFF (5) after continuous splash until first lock/unlock; AOD uses LP1/LP2.
                    if (wantDisplayOn && !powerEventSeen && lastPowerVal == kPanelPowerOff &&
                        dbvVal == 0) {
                        enterDisplayOff(/*disableSensor=*/true);
                    } else if (!wantDisplayOn && !powerEventSeen &&
                               lastPowerVal == kPanelPowerOff && dbvVal > 0) {
                        uint32_t sysfsBl = 0;
                        bool sysfsOk = readPanelSysfsUint("brightness", &sysfsBl);
                        if (!sysfsOk || sysfsBl > 0) {
                            enterDisplayOn();
                        }
                    }
                } else if (!powerEventSeen && lastPowerVal == kPanelPowerOff && dbvVal == 0) {
                    enterDisplayOff(/*disableSensor=*/true);
                }
            }
        }

        isInitialRead = false;

        if (wantDisplayOn && !displayOn && nextEnableRetry &&
            std::chrono::steady_clock::now() >= *nextEnableRetry) {
            enterDisplayOn();
        }

        for (auto& pfd : pollfds) {
            pfd.revents = 0;
        }

        int rc = poll(pollfds.data(), pollfds.size(), computePollTimeoutMs());
        if (rc < 0 && errno != EINTR) {
            PLOG(ERROR) << "failed to poll display nodes";
        }
    }

    enterDisplayOff(/*disableSensor=*/false);
    mEnabled = false;
}
