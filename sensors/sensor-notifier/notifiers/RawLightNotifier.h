/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <android-base/unique_fd.h>

#include <cstdint>
#include <memory>
#include <optional>

#include "SensorNotifier.h"
#include "udals/CwbClient.h"
#include "udals/UDalsBleedCompensator.h"

class RawLightNotifier : public SensorNotifier {
  public:
    explicit RawLightNotifier(sp<ISensorManager> manager);
    ~RawLightNotifier() override;

  protected:
    void notify() override;

  private:
    uint32_t normalizeBacklight(uint32_t raw) const;
    void resolveBacklightScale();
    void updateBacklight(uint32_t rawBacklight);
    bool setSensorEnabled(bool enable);
    void syncCompensatorDisplayState(int dispFd,
                                     std::optional<uint32_t> initialRawBkl = std::nullopt);

    bool mEnabled = false;
    std::optional<uint32_t> mDeviceNormalMaxBacklight;
    android::base::unique_fd mWakeReadFd;
    android::base::unique_fd mWakeWriteFd;
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::shared_ptr<CwbClient> mCwbClient;
};
