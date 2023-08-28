/**
 * A file of general constants
 */

#pragma once

#include <cstddef>

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

namespace GeneralConstants{
  
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