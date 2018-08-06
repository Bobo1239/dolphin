// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/Logging/Log.h"
#include "InputCommon/ControllerInterface/OpenVR/OpenVR.h"

namespace ciface
{
namespace OpenVR
{
void Init()
{
  INFO_LOG(WIIMOTE, "OpenVR::Init");
}

void PopulateDevices()
{
  INFO_LOG(WIIMOTE, "OpenVR::PopulateDevices");
  g_controller_interface.AddDevice(std::make_shared<Device>());
}

void DeInit()
{
  INFO_LOG(WIIMOTE, "OpenVR::DeInit");
}

Device::Device()
{
  AddInput(new Button(0));
}

std::string Device::GetName() const
{
  return "TODO: Device::GetName()";
}

std::string Device::GetSource() const
{
  return "OpenVR";
}

// Update I/O

void Device::UpdateInput()
{
  //INFO_LOG(WIIMOTE, "OpenVR::UpdateInput");
  //PXInputGetState(m_index, &m_state_in);
}

// GET name/source/id

std::string Device::Button::GetName() const
{
  return "TODO: Button::GetName()";
}

// GET / SET STATES

ControlState Device::Button::GetState() const
{
  return true;
}
}
}
