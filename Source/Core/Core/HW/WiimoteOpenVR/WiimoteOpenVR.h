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
}
