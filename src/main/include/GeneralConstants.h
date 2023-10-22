/**
 * A file of general constants
 */

#pragma once

#include <cstddef>
#include <utility>

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
  const vec::Vector2D TAGS[8] = {
    TAG1,
    TAG2,
    TAG3,
    TAG4,
    TAG5,
    TAG6,
    TAG7,
    TAG8
  };

  // different starting positions of robot
  // starting position, starting robot angle (world frame), starting joystick angle
  const vec::Vector2D DEBUG_POS = {0, 0};
  const double DEBUG_ANG = 0;
  const double DEBUG_JANG = 0;

  // TODO find accurate starting values
  // Red left and red middle are close but still need to tune
  const vec::Vector2D BL_POS = {1.9, 4.42};
  const double BL_ANG = M_PI;
  const double BL_JANG = 0;

  const vec::Vector2D BM_POS = {1.9, 2.75};
  const double BM_ANG = M_PI;
  const double BM_JANG = 0;

  const vec::Vector2D BR_POS = {1.9, 1.07};
  const double BR_ANG = M_PI;
  const double BR_JANG = 0;

  const vec::Vector2D RL_POS = {14.5, 1.07};
  const double RL_ANG = 0;
  const double RL_JANG = M_PI;

  const vec::Vector2D RM_POS = {14.5, 2.75};
  const double RM_ANG = 0;
  const double RM_JANG = M_PI;

  const vec::Vector2D RR_POS = {14.5, 4.42};
  const double RR_ANG = 0;
  const double RR_JANG = M_PI;  

  const double FIELD_WIDTH = 16.540988; // in metres

  // TODO TUNE
  // tuned on blue side, tag id 6
  using ScorePair = std::pair<vec::Vector2D, double>;
  const std::vector<std::vector<ScorePair>> BLUE_SCORING_POS {
    { // scoring pos 1
      {{2.07, 4.96}, 0.01}, // L
      {{2.10, 4.88}, 0.12}, // M
      {{1.678, 5.275}, 0.07}, // H
    },
    { // scoring pos 2
      {{2.091, 4.37}, 0.06}, // L
      {{1.734, 4.353}, 0}, // M
      {{1.664, 4.171}, 0}, // H
    },
    { // scoring pos 3
      {{1.965, 3.561}, 0.15}, // L
      {{1.9, 3.842}, 0.09}, // M
      {{1.662, 3.643}, 0.12}, // H
    },
  };
  const double DIST_BETWEEN_TAGS = 1.6764;
}