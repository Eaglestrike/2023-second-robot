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

  const bool FR_INVERTED = 0;
  const bool BR_INVERTED = 0;
  const bool FL_INVERTED = 0;
  const bool BL_INVERTED = 0;

  // degrees, subtracted from reading
  const double FR_OFFSET = -160.9;
  const double BR_OFFSET = 6.0;
  const double FL_OFFSET = -79.4;
  const double BL_OFFSET = -67.5;

  const double TURN_P = 1.3;
  const double TURN_I = 0;
  const double TURN_D = 0;

  const double ANG_CORRECT_P = 20; // ±20 is good number, if you find yourself changing this you're brwoning
  const double ANG_CORRECT_I = 0;
  const double ANG_CORRECT_D = 0.1;

  const double MAX_VOLTS = 6.0; 
}

namespace OdometryConstants {
  const double P_INITIAL = 1.0;
  const double POS_STD_DEV = 0.1;
  const double MEASURE_STD_DEV = 0.1;
  const double CAMERA_TRUST_K = -10.0;
}