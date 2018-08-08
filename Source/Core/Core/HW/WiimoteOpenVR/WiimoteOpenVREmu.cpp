// Copyright 2010 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/WiimoteOpenVR/WiimoteOpenVREmu.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <fstream>
#include <mutex>

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/Config/Config.h"
#include "Common/FileUtil.h"
#include "Common/MathUtil.h"
#include "Common/MsgHandler.h"
#include "Common/Swap.h"

#include "Core/Config/SYSCONFSettings.h"
#include "Core/Config/WiimoteInputSettings.h"
#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/HW/Wiimote.h"
#include "Core/HW/WiimoteCommon/WiimoteConstants.h"
#include "Core/HW/WiimoteCommon/WiimoteHid.h"
#include "Core/HW/WiimoteEmu/MatrixMath.h"
#include "Core/HW/WiimoteEmu/WiimoteEmu.h"
#include "Core/HW/WiimoteReal/WiimoteReal.h"
#include "Core/Movie.h"

// Needed for compilation to succeed; TODO: do we need a comment here?
#include "InputCommon/ControllerEmu/ControlGroup/ControlGroup.h"

namespace
{
// :)
auto const TAU = 6.28318530717958647692;
auto const PI = TAU / 2.0;
}  // namespace

namespace WiimoteOpenVREmu
{
// clang-format off
static const u8 eeprom_data_0[] = {
    // IR, maybe more
    // assuming last 2 bytes are checksum
    0xA1, 0xAA, 0x8B, 0x99, 0xAE, 0x9E, 0x78, 0x30, 0xA7, /*0x74, 0xD3,*/ 0x00,
    0x00,  // messing up the checksum on purpose
    0xA1, 0xAA, 0x8B, 0x99, 0xAE, 0x9E, 0x78, 0x30, 0xA7, /*0x74, 0xD3,*/ 0x00, 0x00,
    // Accelerometer
    // Important: checksum is required for tilt games
    ACCEL_ZERO_G, ACCEL_ZERO_G, ACCEL_ZERO_G, 0, ACCEL_ONE_G, ACCEL_ONE_G, ACCEL_ONE_G, 0, 0, 0xA3,
    ACCEL_ZERO_G, ACCEL_ZERO_G, ACCEL_ZERO_G, 0, ACCEL_ONE_G, ACCEL_ONE_G, ACCEL_ONE_G, 0, 0, 0xA3,
};
// clang-format on

static const u8 motion_plus_id[] = {0x00, 0x00, 0xA6, 0x20, 0x00, 0x05};

static const u8 eeprom_data_16D0[] = {0x00, 0x00, 0x00, 0xFF, 0x11, 0xEE, 0x00, 0x00,
                                      0x33, 0xCC, 0x44, 0xBB, 0x00, 0x00, 0x66, 0x99,
                                      0x77, 0x88, 0x00, 0x00, 0x2B, 0x01, 0xE8, 0x13};

static const WiimoteEmu::ReportFeatures reporting_mode_features[] = {
    // 0x30: Core Buttons
    {2, 0, 0, 0, 4},
    // 0x31: Core Buttons and Accelerometer
    {2, 4, 0, 0, 7},
    // 0x32: Core Buttons with 8 Extension bytes
    {2, 0, 0, 4, 12},
    // 0x33: Core Buttons and Accelerometer with 12 IR bytes
    {2, 4, 7, 0, 19},
    // 0x34: Core Buttons with 19 Extension bytes
    {2, 0, 0, 4, 23},
    // 0x35: Core Buttons and Accelerometer with 16 Extension Bytes
    {2, 4, 0, 7, 23},
    // 0x36: Core Buttons with 10 IR bytes and 9 Extension Bytes
    {2, 0, 4, 14, 23},
    // 0x37: Core Buttons and Accelerometer with 10 IR bytes and 6 Extension Bytes
    {2, 4, 7, 17, 23},

    // UNSUPPORTED:
    // 0x3d: 21 Extension Bytes
    {0, 0, 0, 2, 23},
    // 0x3e / 0x3f: Interleaved Core Buttons and Accelerometer with 36 IR bytes
    {0, 0, 0, 0, 23},
};

// void EmulateShake(AccelData* const accel, ControllerEmu::Buttons* const buttons_group,
//                  const double intensity, u8* const shake_step)
//{
//  // frame count of one up/down shake
//  // < 9 no shake detection in "Wario Land: Shake It"
//  auto const shake_step_max = 15;
//
//  // shake is a bitfield of X,Y,Z shake button states
//  static const unsigned int btns[] = {0x01, 0x02, 0x04};
//  unsigned int shake = 0;
//  buttons_group->GetState(&shake, btns);
//
//  for (int i = 0; i != 3; ++i)
//  {
//    if (shake & (1 << i))
//    {
//      (&(accel->x))[i] = std::sin(TAU * shake_step[i] / shake_step_max) * intensity;
//      shake_step[i] = (shake_step[i] + 1) % shake_step_max;
//    }
//    else
//      shake_step[i] = 0;
//  }
//}

// void EmulateDynamicShake(AccelData* const accel, DynamicData& dynamic_data,
//                         ControllerEmu::Buttons* const buttons_group,
//                         const DynamicConfiguration& config, u8* const shake_step)
//{
//  // frame count of one up/down shake
//  // < 9 no shake detection in "Wario Land: Shake It"
//  auto const shake_step_max = 15;
//
//  // shake is a bitfield of X,Y,Z shake button states
//  static const unsigned int btns[] = {0x01, 0x02, 0x04};
//  unsigned int shake = 0;
//  buttons_group->GetState(&shake, btns);
//
//  for (int i = 0; i != 3; ++i)
//  {
//    if ((shake & (1 << i)) && dynamic_data.executing_frames_left[i] == 0)
//    {
//      dynamic_data.timing[i]++;
//    }
//    else if (dynamic_data.executing_frames_left[i] > 0)
//    {
//      (&(accel->x))[i] = std::sin(TAU * shake_step[i] / shake_step_max) *
//      dynamic_data.intensity[i]; shake_step[i] = (shake_step[i] + 1) % shake_step_max;
//      dynamic_data.executing_frames_left[i]--;
//    }
//    else if (shake == 0 && dynamic_data.timing[i] > 0)
//    {
//      if (dynamic_data.timing[i] > config.frames_needed_for_high_intensity)
//      {
//        dynamic_data.intensity[i] = config.high_intensity;
//      }
//      else if (dynamic_data.timing[i] < config.frames_needed_for_low_intensity)
//      {
//        dynamic_data.intensity[i] = config.low_intensity;
//      }
//      else
//      {
//        dynamic_data.intensity[i] = config.med_intensity;
//      }
//      dynamic_data.timing[i] = 0;
//      dynamic_data.executing_frames_left[i] = config.frames_to_execute;
//    }
//    else
//    {
//      shake_step[i] = 0;
//    }
//  }
//}

// void EmulateTilt(AccelData* const accel, ControllerEmu::Tilt* const tilt_group, const bool
// sideways,
//                 const bool upright)
//{
//  ControlState roll, pitch;
//  // 180 degrees
//  tilt_group->GetState(&roll, &pitch);
//
//  roll *= PI;
//  pitch *= PI;
//
//  unsigned int ud = 0, lr = 0, fb = 0;
//
//  // some notes that no one will understand but me :p
//  // left, forward, up
//  // lr/ left == negative for all orientations
//  // ud/ up == negative for upright longways
//  // fb/ forward == positive for (sideways flat)
//
//  // determine which axis is which direction
//  ud = upright ? (sideways ? 0 : 1) : 2;
//  lr = sideways;
//  fb = upright ? 2 : (sideways ? 0 : 1);
//
//  int sgn[3] = {-1, 1, 1};  // sign fix
//
//  if (sideways && !upright)
//    sgn[fb] *= -1;
//  if (!sideways && upright)
//    sgn[ud] *= -1;
//
//  (&accel->x)[ud] = (sin((PI / 2) - std::max(fabs(roll), fabs(pitch)))) * sgn[ud];
//  (&accel->x)[lr] = -sin(roll) * sgn[lr];
//  (&accel->x)[fb] = sin(pitch) * sgn[fb];
//}

// void EmulateSwing(AccelData* const accel, ControllerEmu::Force* const swing_group,
//                  const double intensity, const bool sideways, const bool upright)
//{
//  ControlState swing[3];
//  swing_group->GetState(swing);
//
//  s8 g_dir[3] = {-1, -1, -1};
//  u8 axis_map[3];
//
//  // determine which axis is which direction
//  axis_map[0] = upright ? (sideways ? 0 : 1) : 2;  // up/down
//  axis_map[1] = sideways;                          // left|right
//  axis_map[2] = upright ? 2 : (sideways ? 0 : 1);  // forward/backward
//
//  // some orientations have up as positive, some as negative
//  // same with forward
//  if (sideways && !upright)
//    g_dir[axis_map[2]] *= -1;
//  if (!sideways && upright)
//    g_dir[axis_map[0]] *= -1;
//
//  for (unsigned int i = 0; i < 3; ++i)
//    (&accel->x)[axis_map[i]] += swing[i] * g_dir[i] * intensity;
//}

// void EmulateDynamicSwing(AccelData* const accel, DynamicData& dynamic_data,
//                         ControllerEmu::Force* const swing_group,
//                         const DynamicConfiguration& config, const bool sideways,
//                         const bool upright)
//{
//  ControlState swing[3];
//  swing_group->GetState(swing);
//
//  s8 g_dir[3] = {-1, -1, -1};
//  u8 axis_map[3];
//
//  // determine which axis is which direction
//  axis_map[0] = upright ? (sideways ? 0 : 1) : 2;  // up/down
//  axis_map[1] = sideways;                          // left|right
//  axis_map[2] = upright ? 2 : (sideways ? 0 : 1);  // forward/backward
//
//  // some orientations have up as positive, some as negative
//  // same with forward
//  if (sideways && !upright)
//    g_dir[axis_map[2]] *= -1;
//  if (!sideways && upright)
//    g_dir[axis_map[0]] *= -1;
//
//  for (unsigned int i = 0; i < 3; ++i)
//  {
//    if (swing[i] > 0 && dynamic_data.executing_frames_left[i] == 0)
//    {
//      dynamic_data.timing[i]++;
//    }
//    else if (dynamic_data.executing_frames_left[i] > 0)
//    {
//      (&accel->x)[axis_map[i]] += g_dir[i] * dynamic_data.intensity[i];
//      dynamic_data.executing_frames_left[i]--;
//    }
//    else if (swing[i] == 0 && dynamic_data.timing[i] > 0)
//    {
//      if (dynamic_data.timing[i] > config.frames_needed_for_high_intensity)
//      {
//        dynamic_data.intensity[i] = config.high_intensity;
//      }
//      else if (dynamic_data.timing[i] < config.frames_needed_for_low_intensity)
//      {
//        dynamic_data.intensity[i] = config.low_intensity;
//      }
//      else
//      {
//        dynamic_data.intensity[i] = config.med_intensity;
//      }
//      dynamic_data.timing[i] = 0;
//      dynamic_data.executing_frames_left[i] = config.frames_to_execute;
//    }
//  }
//}

static const u16 button_bitmasks[] = {
    Wiimote::BUTTON_A,     Wiimote::BUTTON_B,    Wiimote::BUTTON_ONE, Wiimote::BUTTON_TWO,
    Wiimote::BUTTON_MINUS, Wiimote::BUTTON_PLUS, Wiimote::BUTTON_HOME};

static const u16 dpad_bitmasks[] = {Wiimote::PAD_UP, Wiimote::PAD_DOWN, Wiimote::PAD_LEFT,
                                    Wiimote::PAD_RIGHT};
static const u16 dpad_sideways_bitmasks[] = {Wiimote::PAD_RIGHT, Wiimote::PAD_LEFT, Wiimote::PAD_UP,
                                             Wiimote::PAD_DOWN};

static const char* const named_buttons[] = {
    "A", "B", "1", "2", "-", "+", "Home",
};

void Wiimote::Reset()
{
  m_reporting_mode = RT_REPORT_CORE;
  // i think these two are good
  m_reporting_channel = 0;
  m_reporting_auto = false;

  m_rumble_on = false;
  m_speaker_mute = false;

  // will make the first Update() call send a status request
  // the first call to RequestStatus() will then set up the status struct extension bit
  m_need_status_report = true;

  // eeprom
  memset(m_eeprom, 0, sizeof(m_eeprom));
  // calibration data
  memcpy(m_eeprom, eeprom_data_0, sizeof(eeprom_data_0));
  // dunno what this is for, copied from old plugin
  memcpy(m_eeprom + 0x16D0, eeprom_data_16D0, sizeof(eeprom_data_16D0));

  // set up the register
  memset(&m_reg_speaker, 0, sizeof(m_reg_speaker));
  memset(&m_reg_ir, 0, sizeof(m_reg_ir));
  memset(&m_reg_ext, 0, sizeof(m_reg_ext));

  // status
  memset(&m_status, 0, sizeof(m_status));
  // Battery levels in voltage
  //   0x00 - 0x32: level 1
  //   0x33 - 0x43: level 2
  //   0x33 - 0x54: level 3
  //   0x55 - 0xff: level 4
  m_status.battery = 0x66;

  // clear read request queue
  while (!m_read_requests.empty())
  {
    delete[] m_read_requests.front().data;
    m_read_requests.pop();
  }

  // Yamaha ADPCM state initialize
  m_adpcm_state.predictor = 0;
  m_adpcm_state.step = 127;
}

Wiimote::Wiimote(const unsigned int index) : m_index(index), ir_sin(0), ir_cos(1)
{
  // --- reset eeprom/register/values to default ---
  Reset();
}

std::string Wiimote::GetName() const
{
  return std::string("Wiimote OpenVR") + char('1' + m_index);
}

bool Wiimote::Step()
{
  // TODO: use VR controller motor
  // m_motor->control_ref->State(m_rumble_on);

  // when a movie is active, this button status update is disabled (moved), because movies only
  // record data reports.
  if (!Core::WantsDeterminism())
  {
    UpdateButtonsStatus();
  }

  // check if there is a read data request
  if (!m_read_requests.empty())
  {
    ReadRequest& rr = m_read_requests.front();
    // send up to 16 bytes to the Wii
    SendReadDataReply(rr);

    // if there is no more data, remove from queue
    if (0 == rr.size)
    {
      delete[] rr.data;
      m_read_requests.pop();
    }

    // don't send any other reports
    return true;
  }

  // check if a status report needs to be sent
  // this happens on Wii Remote sync and when extensions are switched
  if (m_need_status_report)
  {
    RequestStatus();

    // WiiBrew: Following a connection or disconnection event on the Extension Port,
    // data reporting is disabled and the Data Reporting Mode must be reset before new data can
    // arrive.
    // after a game receives an unrequested status report,
    // it expects data reports to stop until it sets the reporting mode again
    m_reporting_auto = false;

    m_need_status_report = false;

    return true;
  }

  return false;
}

void Wiimote::UpdateButtonsStatus()
{
  // update buttons in status struct
  m_status.buttons.hex = 0;
  // TODO: from controller
  // m_buttons->GetState(&m_status.buttons.hex, button_bitmasks);
  // m_dpad->GetState(&m_status.buttons.hex, is_sideways ? dpad_sideways_bitmasks : dpad_bitmasks);
}

void Wiimote::GetButtonData(u8* const data)
{
  // when a movie is active, the button update happens here instead of Wiimote::Step, to avoid
  // potential desync issues.
  if (Core::WantsDeterminism())
  {
    UpdateButtonsStatus();
  }

  reinterpret_cast<wm_buttons*>(data)->hex |= m_status.buttons.hex;
}

void Wiimote::GetAccelData(u8* const data, const WiimoteEmu::ReportFeatures& rptf)
{
  // const bool sideways_modifier_toggle = m_hotkeys->getSettingsModifier()[0];
  // const bool upright_modifier_toggle = m_hotkeys->getSettingsModifier()[1];
  // const bool sideways_modifier_switch = m_hotkeys->getSettingsModifier()[2];
  // const bool upright_modifier_switch = m_hotkeys->getSettingsModifier()[3];
  // const bool is_sideways =
  //    m_sideways_setting->GetValue() ^ sideways_modifier_toggle ^ sideways_modifier_switch;
  // const bool is_upright =
  //    m_upright_setting->GetValue() ^ upright_modifier_toggle ^ upright_modifier_switch;

  // EmulateSwing(&m_accel, m_swing, Config::Get(Config::WIIMOTE_INPUT_SWING_INTENSITY_MEDIUM),
  //             is_sideways, is_upright);
  // EmulateSwing(&m_accel, m_swing_slow, Config::Get(Config::WIIMOTE_INPUT_SWING_INTENSITY_SLOW),
  //             is_sideways, is_upright);
  // EmulateSwing(&m_accel, m_swing_fast, Config::Get(Config::WIIMOTE_INPUT_SWING_INTENSITY_FAST),
  //             is_sideways, is_upright);
  // EmulateDynamicSwing(&m_accel, m_swing_dynamic_data, m_swing_dynamic, swing_config, is_sideways,
  //                    is_upright);

  // DynamicConfiguration shake_config;
  // shake_config.low_intensity = Config::Get(Config::WIIMOTE_INPUT_SHAKE_INTENSITY_SOFT);
  // shake_config.med_intensity = Config::Get(Config::WIIMOTE_INPUT_SHAKE_INTENSITY_MEDIUM);
  // shake_config.high_intensity = Config::Get(Config::WIIMOTE_INPUT_SHAKE_INTENSITY_HARD);
  // shake_config.frames_needed_for_high_intensity =
  //    Config::Get(Config::WIIMOTE_INPUT_SHAKE_DYNAMIC_FRAMES_HELD_HARD);
  // shake_config.frames_needed_for_low_intensity =
  //    Config::Get(Config::WIIMOTE_INPUT_SHAKE_DYNAMIC_FRAMES_HELD_SOFT);
  // shake_config.frames_to_execute =
  // Config::Get(Config::WIIMOTE_INPUT_SHAKE_DYNAMIC_FRAMES_LENGTH);

  // EmulateShake(&m_accel, m_shake, Config::Get(Config::WIIMOTE_INPUT_SHAKE_INTENSITY_MEDIUM),
  //             m_shake_step.data());
  // EmulateShake(&m_accel, m_shake_soft, Config::Get(Config::WIIMOTE_INPUT_SHAKE_INTENSITY_SOFT),
  //             m_shake_soft_step.data());
  // EmulateShake(&m_accel, m_shake_hard, Config::Get(Config::WIIMOTE_INPUT_SHAKE_INTENSITY_HARD),
  //             m_shake_hard_step.data());
  // EmulateDynamicShake(&m_accel, m_shake_dynamic_data, m_shake_dynamic, shake_config,
  //                    m_shake_dynamic_step.data());

  wm_accel& accel = *reinterpret_cast<wm_accel*>(data + rptf.accel);
  wm_buttons& core = *reinterpret_cast<wm_buttons*>(data + rptf.core);

  // We now use 2 bits more precision, so multiply by 4 before converting to int
  s16 x = (s16)(4 * (m_accel.x * ACCEL_RANGE + ACCEL_ZERO_G));
  s16 y = (s16)(4 * (m_accel.y * ACCEL_RANGE + ACCEL_ZERO_G));
  s16 z = (s16)(4 * (m_accel.z * ACCEL_RANGE + ACCEL_ZERO_G));

  x = 500;
  y = 500;
  z = 500;

  x = MathUtil::Clamp<s16>(x, 0, 1024);
  y = MathUtil::Clamp<s16>(y, 0, 1024);
  z = MathUtil::Clamp<s16>(z, 0, 1024);

  accel.x = (x >> 2) & 0xFF;
  accel.y = (y >> 2) & 0xFF;
  accel.z = (z >> 2) & 0xFF;

  core.acc_x_lsb = x & 0x3;
  core.acc_y_lsb = (y >> 1) & 0x1;
  core.acc_z_lsb = (z >> 1) & 0x1;
}

inline void LowPassFilter(double& var, double newval, double period)
{
  static const double CUTOFF_FREQUENCY = 5.0;

  double RC = 1.0 / CUTOFF_FREQUENCY;
  double alpha = period / (period + RC);
  var = newval * alpha + var * (1.0 - alpha);
}

void Wiimote::GetIRData(u8* const data, bool use_accel)
{
  u16 x[4], y[4];
  memset(x, 0xFF, sizeof(x));

  ControlState xx = 10000, yy = 0, zz = 0;
  double nsin, ncos;

  if (use_accel)
  {
    double ax, az, len;
    ax = m_accel.x;
    az = m_accel.z;
    len = sqrt(ax * ax + az * az);
    if (len)
    {
      ax /= len;
      az /= len;  // normalizing the vector
      nsin = ax;
      ncos = az;
    }
    else
    {
      nsin = 0;
      ncos = 1;
    }
  }
  else
  {
    // TODO m_tilt stuff
    nsin = 0;
    ncos = 1;
  }

  LowPassFilter(ir_sin, nsin, 1.0 / 60);
  LowPassFilter(ir_cos, ncos, 1.0 / 60);

  // TODO
  // m_ir->GetState(&xx, &yy, &zz, true);
  xx = 0;
  yy = 0;
  zz = 0;

  Vertex v[4];

  static const int camWidth = 1024;
  static const int camHeight = 768;
  static const double bndup = -0.315447;
  static const double bnddown = 0.85;
  static const double bndleft = 0.78820266;
  static const double bndright = -0.78820266;
  static const double dist1 = 100.0 / camWidth;  // this seems the optimal distance for zelda
  static const double dist2 = 1.2 * dist1;

  for (auto& vtx : v)
  {
    vtx.x = xx * (bndright - bndleft) / 2 + (bndleft + bndright) / 2;
    if (m_sensor_bar_on_top)
      vtx.y = yy * (bndup - bnddown) / 2 + (bndup + bnddown) / 2;
    else
      vtx.y = yy * (bndup - bnddown) / 2 - (bndup + bnddown) / 2;
    vtx.z = 0;
  }

  v[0].x -= (zz * 0.5 + 1) * dist1;
  v[1].x += (zz * 0.5 + 1) * dist1;
  v[2].x -= (zz * 0.5 + 1) * dist2;
  v[3].x += (zz * 0.5 + 1) * dist2;

#define printmatrix(m)                                                                             \
  PanicAlert("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", m[0][0], m[0][1], m[0][2],    \
             m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3],      \
             m[3][0], m[3][1], m[3][2], m[3][3])
  Matrix rot, tot;
  static Matrix scale;
  MatrixScale(scale, 1, camWidth / camHeight, 1);
  MatrixRotationByZ(rot, ir_sin, ir_cos);
  MatrixMultiply(tot, scale, rot);

  for (int i = 0; i < 4; i++)
  {
    MatrixTransformVertex(tot, v[i]);
    if ((v[i].x < -1) || (v[i].x > 1) || (v[i].y < -1) || (v[i].y > 1))
      continue;
    x[i] = (u16)lround((v[i].x + 1) / 2 * (camWidth - 1));
    y[i] = (u16)lround((v[i].y + 1) / 2 * (camHeight - 1));
  }
  // Fill report with valid data when full handshake was done
  if (m_reg_ir.data[0x30])
    // ir mode
    switch (m_reg_ir.mode)
    {
    // basic
    case 1:
    {
      memset(data, 0xFF, 10);
      wm_ir_basic* const irdata = reinterpret_cast<wm_ir_basic*>(data);
      for (unsigned int i = 0; i < 2; ++i)
      {
        if (x[i * 2] < 1024 && y[i * 2] < 768)
        {
          irdata[i].x1 = static_cast<u8>(x[i * 2]);
          irdata[i].x1hi = x[i * 2] >> 8;

          irdata[i].y1 = static_cast<u8>(y[i * 2]);
          irdata[i].y1hi = y[i * 2] >> 8;
        }
        if (x[i * 2 + 1] < 1024 && y[i * 2 + 1] < 768)
        {
          irdata[i].x2 = static_cast<u8>(x[i * 2 + 1]);
          irdata[i].x2hi = x[i * 2 + 1] >> 8;

          irdata[i].y2 = static_cast<u8>(y[i * 2 + 1]);
          irdata[i].y2hi = y[i * 2 + 1] >> 8;
        }
      }
    }
    break;
    // extended
    case 3:
    {
      memset(data, 0xFF, 12);
      wm_ir_extended* const irdata = reinterpret_cast<wm_ir_extended*>(data);
      for (unsigned int i = 0; i < 4; ++i)
        if (x[i] < 1024 && y[i] < 768)
        {
          irdata[i].x = static_cast<u8>(x[i]);
          irdata[i].xhi = x[i] >> 8;

          irdata[i].y = static_cast<u8>(y[i]);
          irdata[i].yhi = y[i] >> 8;

          irdata[i].size = 10;
        }
    }
    break;
    // full
    case 5:
      PanicAlert("Full IR report");
      // UNSUPPORTED
      break;
    }
}

void Wiimote::GetExtData(u8* const data)
{
  PanicAlert("WiimoteOpenVR has no extensions");
}

void Wiimote::Update()
{
  INFO_LOG(WIIMOTE, "Update() inner");
  // no channel == not connected i guess
  if (0 == m_reporting_channel)
    return;
  INFO_LOG(WIIMOTE, "Update() 1");

  // returns true if a report was sent
  {
    const auto lock = GetStateLock();
    if (Step())
      return;
  }
  INFO_LOG(WIIMOTE, "Update() 2");

  u8 data[MAX_PAYLOAD];
  memset(data, 0, sizeof(data));

  Movie::SetPolledDevice();

  // TODO: get battery level from controller
  // m_status.battery = (u8)(m_battery_setting->GetValue() * 100);
  m_status.battery = 95;

  INFO_LOG(WIIMOTE, "Update() 3");
  const WiimoteEmu::ReportFeatures& rptf =
      reporting_mode_features[m_reporting_mode - RT_REPORT_CORE];
  s8 rptf_size = rptf.size;
  if (Movie::IsPlayingInput() &&
      Movie::PlayWiimote(m_index, data, rptf, WiimoteEmu::EXT_NONE, m_ext_key))
  {
    if (rptf.core)
      m_status.buttons = *reinterpret_cast<wm_buttons*>(data + rptf.core);
  }
  else
  {
    data[0] = 0xA1;
    data[1] = m_reporting_mode;

    const auto lock = GetStateLock();

    // core buttons
    if (rptf.core)
      GetButtonData(data + rptf.core);

    // acceleration
    if (rptf.accel)
      GetAccelData(data, rptf);

    // IR
    if (rptf.ir)
      GetIRData(data + rptf.ir, (rptf.accel != 0));

    // extension
    if (rptf.ext)
      GetExtData(data + rptf.ext);

    Movie::CallWiiInputManip(data, rptf, m_index, WiimoteEmu::EXT_NONE, m_ext_key);
  }

  INFO_LOG(WIIMOTE, "Update() 4");
  Movie::CheckWiimoteStatus(m_index, data, rptf, WiimoteEmu::EXT_NONE, m_ext_key);

  // don't send a data report if auto reporting is off
  if (false == m_reporting_auto && data[1] >= RT_REPORT_CORE)
    return;

  // send data report
  if (rptf_size)
  {
    Core::Callback_WiimoteInterruptChannel(m_index, m_reporting_channel, data, rptf_size);
  }
  INFO_LOG(WIIMOTE, "Update() 5");
}

void Wiimote::ControlChannel(const u16 channel_id, const void* data, u32 size)
{
  // Check for custom communication
  if (99 == channel_id)
  {
    // Wii Remote disconnected
    // reset eeprom/register/reporting mode
    Reset();
    if (WIIMOTE_SRC_REAL == g_wiimote_sources[m_index])
      WiimoteReal::ControlChannel(m_index, channel_id, data, size);
    return;
  }

  // this all good?
  m_reporting_channel = channel_id;

  const hid_packet* hidp = reinterpret_cast<const hid_packet*>(data);

  DEBUG_LOG(WIIMOTE, "Emu ControlChannel (page: %i, type: 0x%02x, param: 0x%02x)", m_index,
            hidp->type, hidp->param);

  switch (hidp->type)
  {
  case HID_TYPE_HANDSHAKE:
    PanicAlert("HID_TYPE_HANDSHAKE - %s", (hidp->param == HID_PARAM_INPUT) ? "INPUT" : "OUPUT");
    break;

  case HID_TYPE_SET_REPORT:
    if (HID_PARAM_INPUT == hidp->param)
    {
      PanicAlert("HID_TYPE_SET_REPORT - INPUT");
    }
    else
    {
      // AyuanX: My experiment shows Control Channel is never used
      // shuffle2: but lwbt uses this, so we'll do what we must :)
      HidOutputReport(reinterpret_cast<const wm_report*>(hidp->data));

      u8 handshake = HID_HANDSHAKE_SUCCESS;
      Core::Callback_WiimoteInterruptChannel(m_index, channel_id, &handshake, 1);
    }
    break;

  case HID_TYPE_DATA:
    PanicAlert("HID_TYPE_DATA - %s", (hidp->param == HID_PARAM_INPUT) ? "INPUT" : "OUTPUT");
    break;

  default:
    PanicAlert("HidControlChannel: Unknown type %x and param %x", hidp->type, hidp->param);
    break;
  }
}

void Wiimote::InterruptChannel(const u16 channel_id, const void* data, u32 size)
{
  // this all good?
  m_reporting_channel = channel_id;

  const hid_packet* hidp = reinterpret_cast<const hid_packet*>(data);

  switch (hidp->type)
  {
  case HID_TYPE_DATA:
    switch (hidp->param)
    {
    case HID_PARAM_OUTPUT:
    {
      const wm_report* sr = reinterpret_cast<const wm_report*>(hidp->data);

      if (WIIMOTE_SRC_REAL == g_wiimote_sources[m_index])
      {
        switch (sr->wm)
        {
        // these two types are handled in RequestStatus() & ReadData()
        case RT_REQUEST_STATUS:
        case RT_READ_DATA:
          if (WIIMOTE_SRC_REAL == g_wiimote_sources[m_index])
            WiimoteReal::InterruptChannel(m_index, channel_id, data, size);
          break;

        default:
          WiimoteReal::InterruptChannel(m_index, channel_id, data, size);
          break;
        }

        HidOutputReport(sr, false);
      }
      else
        HidOutputReport(sr);
    }
    break;

    default:
      PanicAlert("HidInput: HID_TYPE_DATA - param 0x%02x", hidp->param);
      break;
    }
    break;

  default:
    PanicAlert("HidInput: Unknown type 0x%02x and param 0x%02x", hidp->type, hidp->param);
    break;
  }
}

bool Wiimote::CheckForButtonPress()
{
  u16 buttons = 0;
  const auto lock = GetStateLock();
  // TODO
  static bool asd = false;
  asd = !asd;
  if (asd)
  {
    buttons = 1;
  }
  else
  {
    buttons = 0;
  }
  // m_buttons->GetState(&buttons, button_bitmasks);
  // m_dpad->GetState(&buttons, dpad_bitmasks);

  return (buttons != 0);
}

void Wiimote::LoadDefaults(const ControllerInterface& ciface)
{
  EmulatedController::LoadDefaults(ciface);
}

int Wiimote::CurrentExtension() const
{
  return WiimoteEmu::EXT_NONE;
}

bool Wiimote::HaveExtension() const
{
  return false;
}

bool Wiimote::WantExtension() const
{
  return false;
}

void Wiimote::ReportMode(const wm_report_mode* const dr)
{
  // INFO_LOG(WIIMOTE, "Set data report mode");
  // DEBUG_LOG(WIIMOTE, "  Rumble: %x", dr->rumble);
  // DEBUG_LOG(WIIMOTE, "  Continuous: %x", dr->continuous);
  // DEBUG_LOG(WIIMOTE, "  All The Time: %x", dr->all_the_time);
  // DEBUG_LOG(WIIMOTE, "  Mode: 0x%02x", dr->mode);

  // m_reporting_auto = dr->all_the_time;
  m_reporting_auto = dr->continuous;  // this right?
  m_reporting_mode = dr->mode;
  // m_reporting_channel = _channelID;	// this is set in every Interrupt/Control Channel now

  // reset IR camera
  // memset(m_reg_ir, 0, sizeof(*m_reg_ir));  //ugly hack

  if (dr->mode > 0x37)
    PanicAlert("Wiimote: Unsupported Reporting mode.");
  else if (dr->mode < RT_REPORT_CORE)
    PanicAlert("Wiimote: Reporting mode < 0x30.");
}

/* Here we process the Output Reports that the Wii sends. Our response will be
   an Input Report back to the Wii. Input and Output is from the Wii's
   perspective, Output means data to the Wiimote (from the Wii), Input means
   data from the Wiimote.

   The call browser:

   1. Wiimote_InterruptChannel > InterruptChannel > HidOutputReport
   2. Wiimote_ControlChannel > ControlChannel > HidOutputReport

   The IR enable/disable and speaker enable/disable and mute/unmute values are
    bit2: 0 = Disable (0x02), 1 = Enable (0x06)
*/
void Wiimote::HidOutputReport(const wm_report* const sr, const bool send_ack)
{
  DEBUG_LOG(WIIMOTE, "HidOutputReport (page: %i, cid: 0x%02x, wm: 0x%02x)", m_index,
            m_reporting_channel, sr->wm);

  // WiiBrew:
  // In every single Output Report, bit 0 (0x01) of the first byte controls the Rumble feature.
  m_rumble_on = sr->rumble;

  switch (sr->wm)
  {
  case RT_RUMBLE:  // 0x10
    // this is handled above
    return;  // no ack
    break;

  case RT_LEDS:  // 0x11
    // INFO_LOG(WIIMOTE, "Set LEDs: 0x%02x", sr->data[0]);
    m_status.leds = sr->data[0] >> 4;
    break;

  case RT_REPORT_MODE:  // 0x12
    ReportMode(reinterpret_cast<const wm_report_mode*>(sr->data));
    break;

  case RT_IR_PIXEL_CLOCK:  // 0x13
    // INFO_LOG(WIIMOTE, "WM IR Clock: 0x%02x", sr->data[0]);
    // m_ir_clock = sr->enable;
    if (false == sr->ack)
      return;
    break;

  case RT_SPEAKER_ENABLE:  // 0x14
    // ERROR_LOG(WIIMOTE, "WM Speaker Enable: %02x", sr->enable);
    // PanicAlert( "WM Speaker Enable: %d", sr->data[0] );
    m_status.speaker = sr->enable;
    if (false == sr->ack)
      return;
    break;

  case RT_REQUEST_STATUS:  // 0x15
    if (WIIMOTE_SRC_EMU == g_wiimote_sources[m_index])
      RequestStatus(reinterpret_cast<const wm_request_status*>(sr->data));
    return;  // sends its own ack
    break;

  case RT_WRITE_DATA:  // 0x16
    WriteData(reinterpret_cast<const wm_write_data*>(sr->data));
    break;

  case RT_READ_DATA:  // 0x17
    if (WIIMOTE_SRC_EMU == g_wiimote_sources[m_index])
      ReadData(reinterpret_cast<const wm_read_data*>(sr->data));
    return;  // sends its own ack
    break;

  case RT_WRITE_SPEAKER_DATA:  // 0x18
    if (WIIMOTE_SRC_EMU == g_wiimote_sources[m_index] && !m_speaker_mute)
      // TODO
      // Wiimote::SpeakerData(reinterpret_cast<const wm_speaker_data*>(sr->data));
    return;  // no ack
    break;

  case RT_SPEAKER_MUTE:  // 0x19
    m_speaker_mute = sr->enable;
    if (false == sr->ack)
      return;
    break;

  case RT_IR_LOGIC:  // 0x1a
    // comment from old plugin:
    // This enables or disables the IR lights, we update the global variable g_IR
    // so that WmRequestStatus() knows about it
    m_status.ir = sr->enable;
    if (false == sr->ack)
      return;
    break;

  default:
    PanicAlert("HidOutputReport: Unknown channel 0x%02x", sr->wm);
    return;  // no ack
    break;
  }

  // send ack
  if (send_ack && WIIMOTE_SRC_EMU == g_wiimote_sources[m_index])
    SendAck(sr->wm);
}

/* This will generate the 0x22 acknowledgement for most Input reports.
   It has the form of "a1 22 00 00 _reportID 00".
   The first two bytes are the core buttons data,
   00 00 means nothing is pressed.
   The last byte is the success code 00. */
void Wiimote::SendAck(u8 report_id)
{
  u8 data[6];

  data[0] = 0xA1;
  data[1] = RT_ACK_DATA;

  wm_acknowledge* ack = reinterpret_cast<wm_acknowledge*>(data + 2);

  ack->buttons = m_status.buttons;
  ack->reportID = report_id;
  ack->errorID = 0;

  Core::Callback_WiimoteInterruptChannel(m_index, m_reporting_channel, data, sizeof(data));
}

void Wiimote::HandleExtensionSwap()
{
  assert(false);
}

void Wiimote::RequestStatus(const wm_request_status* const rs)
{
  HandleExtensionSwap();

  // update status struct
  m_status.extension = 0;

  // set up report
  u8 data[8];
  data[0] = 0xA1;
  data[1] = RT_STATUS_REPORT;

  // status values
  *reinterpret_cast<wm_status_report*>(data + 2) = m_status;

  // hybrid Wiimote stuff
  if (WIIMOTE_SRC_REAL == g_wiimote_sources[m_index] && m_need_status_report)
  {
    using namespace WiimoteReal;

    std::lock_guard<std::mutex> lk(g_wiimotes_mutex);

    if (g_wiimotes[m_index])
    {
      wm_request_status rpt = {};
      g_wiimotes[m_index]->QueueReport(RT_REQUEST_STATUS, &rpt, sizeof(rpt));
    }

    m_need_status_report = false;

    return;
  }

  // send report
  Core::Callback_WiimoteInterruptChannel(m_index, m_reporting_channel, data, sizeof(data));
}

/* Write data to Wiimote and Extensions registers. */
void Wiimote::WriteData(const wm_write_data* const wd)
{
  u32 address = Common::swap24(wd->address);

  // ignore the 0x010000 bit
  address &= ~0x010000;

  if (wd->size > 16)
  {
    PanicAlert("WriteData: size is > 16 bytes");
    return;
  }

  switch (wd->space)
  {
  case WS_EEPROM:
  {
    // Write to EEPROM

    if (address + wd->size > WIIMOTE_EEPROM_SIZE)
    {
      ERROR_LOG(WIIMOTE, "WriteData: address + size out of bounds!");
      PanicAlert("WriteData: address + size out of bounds!");
      return;
    }
    memcpy(m_eeprom + address, wd->data, wd->size);

    // write mii data to file
    if (address >= 0x0FCA && address < 0x12C0)
    {
      // TODO Only write parts of the Mii block
      std::ofstream file;
      File::OpenFStream(file, File::GetUserPath(D_SESSION_WIIROOT_IDX) + "/mii.bin",
                        std::ios::binary | std::ios::out);
      file.write((char*)m_eeprom + 0x0FCA, 0x02f0);
      file.close();
    }
  }
  break;

  case WS_REGS1:
  case WS_REGS2:
  {
    // Write to Control Register

    // ignore second byte for extension area
    if (0xA4 == (address >> 16))
      address &= 0xFF00FF;

    const u8 region_offset = (u8)address;
    void* region_ptr = nullptr;
    int region_size = 0;

    switch (address >> 16)
    {
    // speaker
    case 0xa2:
      region_ptr = &m_reg_speaker;
      region_size = WIIMOTE_REG_SPEAKER_SIZE;
      break;

    // extension register
    case 0xa4:
      region_ptr = (void*)&m_reg_ext;
      region_size = WIIMOTE_REG_EXT_SIZE;
      break;

    // ir
    case 0xB0:
      region_ptr = &m_reg_ir;
      region_size = WIIMOTE_REG_IR_SIZE;
      break;
    }

    if (region_ptr && (region_offset + wd->size <= region_size))
    {
      memcpy((u8*)region_ptr + region_offset, wd->data, wd->size);
    }
    else
      return;  // TODO: generate a writedata error reply

    if (&m_reg_ext == region_ptr)
    {
      // Run the key generation on all writes in the key area, it doesn't matter
      // that we send it parts of a key, only the last full key will have an effect
      if (address >= 0xa40040 && address <= 0xa4004c)
        WiimoteGenerateKey(&m_ext_key, m_reg_ext.encryption_key);
    }
  }
  break;

  default:
    PanicAlert("WriteData: unimplemented parameters!");
    break;
  }
}

/* Read data from Wiimote and Extensions registers. */
void Wiimote::ReadData(const wm_read_data* const rd)
{
  u32 address = Common::swap24(rd->address);
  u16 size = Common::swap16(rd->size);

  // ignore the 0x010000 bit
  address &= 0xFEFFFF;

  // hybrid Wiimote stuff
  // relay the read data request to real-Wiimote
  if (WIIMOTE_SRC_REAL == g_wiimote_sources[m_index] &&
      ((0xA4 != (address >> 16)) || m_need_status_report))
  {
    WiimoteReal::InterruptChannel(m_index, m_reporting_channel, ((u8*)rd) - 2,
                                  sizeof(wm_read_data) + 2);  // hacky

    // don't want emu-Wiimote to send reply
    return;
  }

  ReadRequest rr;
  u8* const block = new u8[size];

  switch (rd->space)
  {
  case WS_EEPROM:
  {
    // Read from EEPROM
    if (address + size >= WIIMOTE_EEPROM_FREE_SIZE)
    {
      if (address + size > WIIMOTE_EEPROM_SIZE)
      {
        PanicAlert("ReadData: address + size out of bounds");
        delete[] block;
        return;
      }
      // generate a read error
      size = 0;
    }

    // read mii data from file
    if (address >= 0x0FCA && address < 0x12C0)
    {
      // TODO Only read the Mii block parts required
      std::ifstream file;
      File::OpenFStream(file, (File::GetUserPath(D_SESSION_WIIROOT_IDX) + "/mii.bin").c_str(),
                        std::ios::binary | std::ios::in);
      file.read((char*)m_eeprom + 0x0FCA, 0x02f0);
      file.close();
    }

    // read memory to be sent to Wii
    memcpy(block, m_eeprom + address, size);
  }
  break;

  case WS_REGS1:
  case WS_REGS2:
  {
    // Read from Control Register

    // ignore second byte for extension area
    if (0xA4 == (address >> 16))
      address &= 0xFF00FF;

    const u8 region_offset = (u8)address;
    void* region_ptr = nullptr;
    int region_size = 0;

    switch (address >> 16)
    {
    // speaker
    case 0xa2:
      region_ptr = &m_reg_speaker;
      region_size = WIIMOTE_REG_SPEAKER_SIZE;
      break;

    // extension
    case 0xa4:
      region_ptr = (void*)&m_reg_ext;
      region_size = WIIMOTE_REG_EXT_SIZE;
      break;

    // ir
    case 0xb0:
      region_ptr = &m_reg_ir;
      region_size = WIIMOTE_REG_IR_SIZE;
      break;
    }

    if (region_ptr && (region_offset + size <= region_size))
    {
      memcpy(block, (u8*)region_ptr + region_offset, size);
    }
    else
      size = 0;  // generate read error

    if (&m_reg_ext == region_ptr)
    {
      // Encrypt data read from extension register
      // Check if encrypted reads is on
      if (0xaa == m_reg_ext.encryption)
        WiimoteEncrypt(&m_ext_key, block, address & 0xffff, (u8)size);
    }
  }
  break;

  default:
    PanicAlert("WmReadData: unimplemented parameters (size: %i, address: 0x%x)!", size, rd->space);
    break;
  }

  // want the requested address, not the above modified one
  rr.address = Common::swap24(rd->address);
  rr.size = size;
  // rr.channel = _channelID;
  rr.position = 0;
  rr.data = block;

  // send up to 16 bytes
  SendReadDataReply(rr);

  // if there is more data to be sent, add it to the queue
  if (rr.size)
    m_read_requests.push(rr);
  else
    delete[] rr.data;
}

void Wiimote::SendReadDataReply(ReadRequest& request)
{
  u8 data[23];
  data[0] = 0xA1;
  data[1] = RT_READ_DATA_REPLY;

  wm_read_data_reply* const reply = reinterpret_cast<wm_read_data_reply*>(data + 2);
  reply->buttons = m_status.buttons;
  reply->address = Common::swap16(request.address);

  // generate a read error
  // Out of bounds. The real Wiimote generate an error for the first
  // request to 0x1770 if we dont't replicate that the game will never
  // read the calibration data at the beginning of Eeprom. I think this
  // error is supposed to occur when we try to read above the freely
  // usable space that ends at 0x16ff.
  if (0 == request.size)
  {
    reply->size = 0x0f;
    reply->error = 0x08;

    memset(reply->data, 0, sizeof(reply->data));
  }
  else
  {
    // Limit the amt to 16 bytes
    // AyuanX: the MTU is 640B though... what a waste!
    const int amt = std::min((unsigned int)16, request.size);

    // no error
    reply->error = 0;

    // 0x1 means two bytes, 0xf means 16 bytes
    reply->size = amt - 1;

    // Clear the mem first
    memset(reply->data, 0, sizeof(reply->data));

    // copy piece of mem
    memcpy(reply->data, request.data + request.position, amt);

    // update request struct
    request.size -= amt;
    request.position += amt;
    request.address += amt;
  }

  // Send a piece
  Core::Callback_WiimoteInterruptChannel(m_index, m_reporting_channel, data, sizeof(data));
}

void Wiimote::DoState(PointerWrap& p)
{
  p.Do(m_accel);
  p.Do(m_index);
  p.Do(ir_sin);
  p.Do(ir_cos);
  p.Do(m_rumble_on);
  p.Do(m_speaker_mute);
  p.Do(m_reporting_auto);
  p.Do(m_reporting_mode);
  p.Do(m_reporting_channel);
  p.Do(m_shake_step);
  p.Do(m_sensor_bar_on_top);
  p.Do(m_status);
  p.Do(m_adpcm_state);
  p.Do(m_ext_key);
  p.DoArray(m_eeprom);
  p.Do(m_reg_ir);
  p.Do(m_reg_ext);
  p.Do(m_reg_speaker);

  // Do 'm_read_requests' queue
  {
    u32 size = 0;
    if (p.mode == PointerWrap::MODE_READ)
    {
      // clear
      while (!m_read_requests.empty())
      {
        delete[] m_read_requests.front().data;
        m_read_requests.pop();
      }

      p.Do(size);
      while (size--)
      {
        ReadRequest tmp;
        p.Do(tmp.address);
        p.Do(tmp.position);
        p.Do(tmp.size);
        tmp.data = new u8[tmp.size];
        p.DoArray(tmp.data, tmp.size);
        m_read_requests.push(tmp);
      }
    }
    else
    {
      std::queue<ReadRequest> tmp_queue(m_read_requests);
      size = (u32)(m_read_requests.size());
      p.Do(size);
      while (!tmp_queue.empty())
      {
        ReadRequest tmp = tmp_queue.front();
        p.Do(tmp.address);
        p.Do(tmp.position);
        p.Do(tmp.size);
        p.DoArray(tmp.data, tmp.size);
        tmp_queue.pop();
      }
    }
  }
  p.DoMarker("Wiimote");

  if (p.GetMode() == PointerWrap::MODE_READ)
    RealState();
}

// load real Wiimote state
void Wiimote::RealState()
{
  using namespace WiimoteReal;

  if (g_wiimotes[m_index])
  {
    g_wiimotes[m_index]->SetChannel(m_reporting_channel);
    g_wiimotes[m_index]->EnableDataReporting(m_reporting_mode);
  }
}
}
