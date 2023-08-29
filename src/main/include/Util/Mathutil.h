#pragma once

#include <vector>

#include "UtilConstants.h"
#include "thirdparty/simplevectors.hpp"

namespace vec = svector; //!< vector namespace alias

/**
 * Math utility class with static methods
*/
class Mathutil {
public:
  static double AbsMin(const double a, const double b);
  static vec::Vector2D GetVecAverage(const std::vector<vec::Vector2D>);
  static bool NearZero(const double num, const double tolerance = UtilConstants::NEAR_ZERO_TOLERANCE);
  static bool NearZero(const vec::Vector2D vec, const double tolerance = UtilConstants::NEAR_ZERO_TOLERANCE);
};