// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "InputCommon/ControllerInterface/ControllerInterface.h"

namespace ciface
{
namespace OpenVR
{
void Init();
void PopulateDevices();
void DeInit();

class Device : public Core::Device
{
private:
  class Button : public Core::Device::Input
  {
  public:
    Button(u8 index) : m_index(index) {}
    std::string GetName() const override;
    ControlState GetState() const override;

  private:
    u8 m_index;
  };

public:
  void UpdateInput() override;

  Device();

  std::string GetName() const override;
  std::string GetSource() const override;

private:
};
}
}
