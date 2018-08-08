// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "Core/HW/WiimoteOpenVR/WiimoteOpenVREmu.h"

#include "Core/HW/Wiimote.h"

namespace WiimoteOpenVR
{
// TODO: unique_ptr?
extern WiimoteOpenVREmu::Wiimote* g_wiimoteas[MAX_WIIMOTES];

void Initialize();
void Pause();
void Resume();
void Stop();

void Update(int wiimote_number);
bool CheckForButtonPress(int wiimote_number);
}
