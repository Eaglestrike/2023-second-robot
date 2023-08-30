#pragma once

#include <cstddef>
#include <vector>

#include "UtilConstants.h"
#include "thirdparty/simplevectors.hpp"

namespace vec = svector; //!< vector namespace alias

/**
 * Utility class with static methods
 * 
 * Math + time included
*/
class Utils {
public:
  static double AbsMin(const double a, const double b);
  static vec::Vector2D GetVecAverage(const std::vector<vec::Vector2D>);
  static bool NearZero(const double num, const double tolerance = UtilConstants::NEAR_ZERO_TOLERANCE);
  static bool NearZero(const vec::Vector2D vec, const double tolerance = UtilConstants::NEAR_ZERO_TOLERANCE);
  static double NormalizeAng(const double ang);
  static std::size_t GetCurTimeMs();
  static double GetCurTimeS();
  static double DegToRad(const double deg);
  static double RadToDeg(const double rad);
};