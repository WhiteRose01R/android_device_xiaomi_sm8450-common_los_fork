/*
 * Copyright (C) 2024-2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

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
    uint32_t normalizeBacklight(uint32_t raw) const;

    bool mEnabled;
    const uint32_t mNormalMaxBacklight;
    std::shared_ptr<UDalsBleedCompensator> mCompensator;
    std::unique_ptr<CwbClient> mCwbClient;
};
