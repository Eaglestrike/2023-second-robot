/**
 * A file of constants for the drivebase
 */

#pragma once

#include <cstddef>

namespace SwerveConstants
{
  const double MAG_ENCODER_COUNTS_PER_REV = 4096;
  const double TALON_FX_COUNTS_PER_REV = 2048;
  const double WHEEL_RADIUS = 0.0508; // in meters
  const double WHEEL_GEAR_RATIO = 6.12; // stolen from Alex, 6.12 motor spins = 1 wheel spin

  // meters
  const double CENTER_TO_EDGE = 0.368;

  const std::size_t FR_DRIVE_ID = 14;
  const std::size_t BR_DRIVE_ID = 11;
  const std::size_t FL_DRIVE_ID = 21;
  const std::size_t BL_DRIVE_ID = 17;

  const std::size_t FR_TURN_ID = 13;
  const std::size_t BR_TURN_ID = 12;
  const std::size_t FL_TURN_ID = 15;
  const std::size_t BL_TURN_ID = 18;

  const std::size_t FR_ENCODER_ID = 42;
  const std::size_t BR_ENCODER_ID = 10;
  const std::size_t FL_ENCODER_ID = 62;
  const std::size_t BL_ENCODER_ID = 8;

  // If positive drive motor does not move swerve module forward when angle is 0
  const bool FR_DRIVE_INVERTED = false;
  const bool BR_DRIVE_INVERTED = false;
  const bool FL_DRIVE_INVERTED = false;
  const bool BL_DRIVE_INVERTED = false;

  // if positive encoder != CCW movement
  const bool FR_ENCODER_INVERTED = false;
  const bool BR_ENCODER_INVERTED = false;
  const bool FL_ENCODER_INVERTED = false;
  const bool BL_ENCODER_INVERTED = false;

  // if positive angle motor != CCW movement
  const bool FR_ANG_INVERTED = true;
  const bool BR_ANG_INVERTED = true;
  const bool FL_ANG_INVERTED = true;
  const bool BL_ANG_INVERTED = true;

  // degrees, subtracted from reading
  const double FR_OFFSET = -161.28; //18.72
  const double BR_OFFSET = 6.50;
  const double FL_OFFSET = 104.4;
  const double BL_OFFSET = 108.95;

  const double TURN_P = 4.0;
  const double TURN_I = 0;
  const double TURN_D = 0.0;

  const double ANG_CORRECT_P = 15;
  const double ANG_CORRECT_I = 0;
  const double ANG_CORRECT_D = 0.1;

  const double MAX_VOLTS = 6.0; 
}

namespace OdometryConstants {
const double E0 = 1.0;
  const double Q = 0.01;
  const double CAM_TRUST_KANG = 10.0; // unused for now, can use if relying on apriltag angle
  const double CAM_TRUST_KPOS = 500.0;
  const double CAM_TRUST_KPOSINT = 100.0;
  const double MAX_TIME = 0.5;
}