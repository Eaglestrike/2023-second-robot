#pragma once

#include <cstddef>

/**
 * A file of constants for the drivebase
 */

namespace SwerveConstants
{
  const double MAG_ENCODER_COUNTS_PER_REV = 4096;
  const double TALON_FX_COUNTS_PER_REV = 2048;
  const double WHEEL_RADIUS = 0.0508; // in meters
  const double WHEEL_GEAR_RATIO = 6.12; // stolen from Alex, 6.12 motor spins = 1 wheel spin

  // meters
  const double CENTER_TO_EDGE = 0.368;

  const std::size_t FR_DRIVE_ID = 4;
  const std::size_t BR_DRIVE_ID = 1;
  const std::size_t FL_DRIVE_ID = 23;
  const std::size_t BL_DRIVE_ID = 22;

  const std::size_t FR_TURN_ID = 10;
  const std::size_t BR_TURN_ID = 7;
  const std::size_t FL_TURN_ID = 5;
  const std::size_t BL_TURN_ID = 19;

  const std::size_t FR_ENCODER_ID = 8;
  const std::size_t BR_ENCODER_ID = 9;
  const std::size_t FL_ENCODER_ID = 2;
  const std::size_t BL_ENCODER_ID = 6;

  const bool FR_INVERTED = 0;
  const bool BR_INVERTED = 1;
  const bool FL_INVERTED = 0;
  const bool BL_INVERTED = 1;

  // degrees
  const double FR_OFFSET = 12.5;
  const double BR_OFFSET = -148.7;
  const double FL_OFFSET = 7.29;
  const double BL_OFFSET = -83.7;

  const double TURN_P = 3.5;
  const double TURN_I = 0;
  const double TURN_D = 0;

  const double ANG_CORRECT_P = 20; // ±20 is good number, if you find yourself changing this you're brwoning
  const double ANG_CORRECT_I = 0;
  const double ANG_CORRECT_D = 0.1;

  const double MAX_VOLTS = 12.0; 
}

namespace OdometryConstants {
  const double P_INITIAL = 1.0;
  const double POS_STD_DEV = 0.1;
  const double MEASURE_STD_DEV = 0.1;
  const double CAMERA_TRUST_K = -10.0;
}