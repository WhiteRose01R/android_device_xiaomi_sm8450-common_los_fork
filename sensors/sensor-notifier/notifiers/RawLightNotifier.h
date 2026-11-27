/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <android-base/unique_fd.h>

#include "SensorNotifier.h"

class RawLightNotifier : public SensorNotifier {
  public:
    explicit RawLightNotifier(sp<ISensorManager> manager);
    ~RawLightNotifier() override;

  protected:
    void notify() override;

  private:
    bool setSensorEnabled(bool enable);

    bool mEnabled = false;
    android::base::unique_fd mWakeReadFd;
    android::base::unique_fd mWakeWriteFd;
};
