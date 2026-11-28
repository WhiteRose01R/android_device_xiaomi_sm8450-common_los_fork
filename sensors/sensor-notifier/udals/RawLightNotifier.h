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

#include "CwbClient.h"
#include "SensorNotifier.h"
#include "UDalsBleedCompensator.h"

class RawLightNotifier : public SensorNotifier {
  public:
    explicit RawLightNotifier(sp<ISensorManager> manager);
    ~RawLightNotifier() override;

  protected:
    void notify() override;

  private:
    void updateBacklight(uint32_t rawBacklight);
    bool setSensorEnabled(bool enable);
    void syncCompensatorDisplayState(int dbvFd,
                                     std::optional<uint32_t> initialRawDbv = std::nullopt);

    bool mEnabled = false;
    int32_t mSamplingPeriodUs = 20000;
    android::base::unique_fd mWakeReadFd;
    android::base::unique_fd mWakeWriteFd;
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::shared_ptr<CwbClient> mCwbClient;
};
