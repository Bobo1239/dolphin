// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

namespace WiimoteOpenVR
{
class Wiimote
{
public:
  Wiimote();
  virtual ~Wiimote() {}
};
}

namespace WiimoteOpenVR
{
void Initialize();
void Pause();
void Resume();
void Stop();

void Update(int wiimote_number);
bool CheckForButtonPress(int wiimote_number);
}
