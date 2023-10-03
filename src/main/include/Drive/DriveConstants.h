/**
 * A file of constants for the drivebase
 */

#pragma once

#include <cstddef>
#include <string>

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

namespace SwerveConstants{
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

  // encoder offset degrees, subtracted from reading
  const double FR_OFFSET = -161.28; //18.72
  const double BR_OFFSET = 6.50;
  const double FL_OFFSET = 104.4;
  const double BL_OFFSET = 108.95;

  const double TURN_P = 4.0;
  const double TURN_I = 0;
  const double TURN_D = 0.0;

  const double ANG_CORRECT_P = 20; // ±20 is good number, if you find yourself changing this you're brwoning
  const double ANG_CORRECT_I = 0;
  const double ANG_CORRECT_D = 0.1;

  const double MAX_VOLTS = 6.0; 

  struct SwerveConfig{
    std::string name;
    std::size_t driveMotorId;
    std::size_t angleMotorId;
    std::size_t encoderId;
    bool driveInverted;
    bool encoderInverted;
    bool angMotorInverted;
    double offset;
    vec::Vector2D position;
    double kP = TURN_P;
    double kI = TURN_I;
    double kD = TURN_D;
  };

  const SwerveConfig FR_CONFIG{
     .name = "Front Right",
     .driveMotorId = FR_DRIVE_ID, 
     .angleMotorId = FR_TURN_ID,
     .encoderId = FR_ENCODER_ID,
     .driveInverted = FR_DRIVE_INVERTED,
     .encoderInverted = FR_ENCODER_INVERTED,
     .angMotorInverted = FR_ANG_INVERTED,
     .offset = FR_OFFSET,
     .position = {SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE}
  };

  const SwerveConfig BR_CONFIG{
     .name = "Back Right",
     .driveMotorId = BR_DRIVE_ID, 
     .angleMotorId = BR_TURN_ID,
     .encoderId = BR_ENCODER_ID,
     .driveInverted = BR_DRIVE_INVERTED,
     .encoderInverted = BR_ENCODER_INVERTED,
     .angMotorInverted = BR_ANG_INVERTED,
     .offset = BR_OFFSET,
     .position = {-SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE}
  };

  const SwerveConfig FL_CONFIG{
     .name = "Front Left",
     .driveMotorId = FL_DRIVE_ID, 
     .angleMotorId = FL_TURN_ID,
     .encoderId = FL_ENCODER_ID,
     .driveInverted = FL_DRIVE_INVERTED,
     .encoderInverted = FL_ENCODER_INVERTED,
     .angMotorInverted = FL_ANG_INVERTED,
     .offset = FL_OFFSET,
     .position = {SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE}
  };

  const SwerveConfig BL_CONFIG{
     .name = "Back Left",
     .driveMotorId = BL_DRIVE_ID, 
     .angleMotorId = BL_TURN_ID,
     .encoderId = BL_ENCODER_ID,
     .driveInverted = BL_DRIVE_INVERTED,
     .encoderInverted = BL_ENCODER_INVERTED,
     .angMotorInverted = BL_ANG_INVERTED,
     .offset = BL_OFFSET,
     .position = {-SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE}
  };
}

namespace OdometryConstants {
  const double P_INITIAL = 1.0;
  const double POS_STD_DEV = 0.1;
  const double MEASURE_STD_DEV = 0.1;
  const double CAMERA_TRUST_K = -10.0;
}