/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>
#include <memory>

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
    void applyNormalizedBacklight(uint32_t normalized);
    uint32_t normalizeBacklight(uint32_t raw) const;
    void resolveBacklightScale();
    void updateBacklight(uint32_t rawBacklight);
    bool setSensorEnabled(bool enable);
    void syncCompensatorDisplayState(int dispFd);

    std::atomic<bool> mEnabled;
    bool mBacklightScaleResolved;
    uint32_t mDeviceNormalMaxBacklight;
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::shared_ptr<CwbClient> mCwbClient;
};
