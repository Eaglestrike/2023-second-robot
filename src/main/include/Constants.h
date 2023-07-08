/**
 * A file of constants
 *
 * @note General constants are not namespaced
 */

#pragma once

#include <cstddef>

#include "thirdparty/simplevectors.hpp"

namespace vec = svector;

const double NEAR_ZERO_TOLERANCE = 0.000001;

namespace SwerveConstants
{
  const double MAG_ENCODER_COUNTS_PER_REV = 4096;
  const double TALON_FX_COUNTS_PER_REV = 2048;
  const double WHEEL_RADIUS = 0.0508; // in meters
  const double WHEEL_GEAR_RATIO = 6.12; // stolen from Alex, 6.12 motor spins = 1 wheel spin

  // meters
  const double CENTER_TO_EDGE = 0.368; // grid distance from center to wheel

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
  const double E0 = 1.0;
  const double Q = 0.01;
  const double CAM_TRUST_KANG = 10.0; // unused for now, can use if relying on apriltag angle
  const double CAM_TRUST_KPOS = 500.0;
  const double CAM_TRUST_KPOSINT = 100.0;
  const double MAX_TIME = 0.5;
}

namespace FieldConstants {
  // positions of tags, in m
  const vec::Vector2D TAG1 = {15.513558, 1.071626};
  const vec::Vector2D TAG2 = {15.513558, 2.748026};
  const vec::Vector2D TAG3 = {15.513558, 4.424426};
  const vec::Vector2D TAG4 = {16.178794, 6.749796};
  const vec::Vector2D TAG5 = {0.36195, 6.749796};
  const vec::Vector2D TAG6 = {1.02743, 4.424426};
  const vec::Vector2D TAG7 = {1.02743, 2.748026};
  const vec::Vector2D TAG8 = {1.02743, 1.071626};

  // different starting positions of robot
  // starting position, starting robot angle (world frame), starting joystick angle
  const vec::Vector2D DEBUG_POS = {0, 0};
  const double DEBUG_ANG = 0;
  const double DEBUG_JANG = 0;

  // TODO find accurate starting values
  // Red left and red middle are close but still need to tune
  const vec::Vector2D BL_POS = {0, 5};
  const double BL_ANG = M_PI;
  const double BL_JANG = 0;

  const vec::Vector2D BM_POS = {0, 3};
  const double BM_ANG = M_PI;
  const double BM_JANG = 0;

  const vec::Vector2D BR_POS = {0, 1};
  const double BR_ANG = M_PI;
  const double BR_JANG = 0;

  const vec::Vector2D RL_POS = {14.35, 1};
  const double RL_ANG = 0;
  const double RL_JANG = M_PI;

  const vec::Vector2D RM_POS = {14.35, 2.5};
  const double RM_ANG = 0;
  const double RM_JANG = M_PI;

  const vec::Vector2D RR_POS = {10, 5};
  const double RR_ANG = 0;
  const double RR_JANG = M_PI;
}

namespace RobotConstants {
  // meters
  // first robot is 33.5in x 34in
  const double WIDTH = 0.85;
  const double LENGTH = 0.86;
}