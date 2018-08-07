// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/Logging/Log.h"

namespace WiimoteOpenVR
{
void Initialize()
{
  INFO_LOG(WIIMOTE, "Initialize()");
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
}

bool CheckForButtonPress(int wiimote_number)
{
  INFO_LOG(WIIMOTE, "CheckForButtonPress()");
  static bool asd = false;
  return asd = !asd;
}
}
