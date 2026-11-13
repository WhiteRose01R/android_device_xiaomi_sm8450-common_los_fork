/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <atomic>

#include "SensorNotifier.h"

class RawLightNotifier : public SensorNotifier {
  public:
    explicit RawLightNotifier(sp<ISensorManager> manager);
    ~RawLightNotifier() override;

  protected:
    void notify() override;

  private:
    std::atomic<bool> mEnabled;
};
