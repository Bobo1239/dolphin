// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "WiimoteOpenVR.h"

#include "Common/Logging/Log.h"

#include "Core/HW/WiimoteOpenVR/WiimoteOpenVREmu.h"

namespace WiimoteOpenVR
{
WiimoteOpenVREmu::Wiimote* g_wiimoteas[MAX_WIIMOTES];

void Initialize()
{
  INFO_LOG(WIIMOTE, "Initialize()");
  WiimoteOpenVREmu::Wiimote wiimote(0);
  g_wiimoteas[0] = &wiimote;
}

void Pause()
{
  INFO_LOG(WIIMOTE, "Pause()");
}

void Resume()
{
  INFO_LOG(WIIMOTE, "Resume()");
}

void Stop()
{
  INFO_LOG(WIIMOTE, "Stop()");
}

void Update(int wiimote_number)
{
  INFO_LOG(WIIMOTE, "Update()");
  g_wiimoteas[0]->Update();
}

bool CheckForButtonPress(int wiimote_number)
{
  INFO_LOG(WIIMOTE, "CheckForButtonPress()");
  static bool asd = false;
  return asd = !asd;
}
}
