#pragma once

#include <cstddef>
#include <vector>

#include "Drive/DriveConstants.h"
#include "GeneralConstants.h"
#include "thirdparty/simplevectors.hpp"
#include "Util/UtilConstants.h"

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
  static vec::Vector2D GetUnitVecDir(const double ang);
  static vec::Vector2D GetProjection(const vec::Vector2D v, const vec::Vector2D w);
  static double GetAngBetweenVec(const vec::Vector2D v1, const vec::Vector2D v2);
  static AutoPaths::SwervePose GetRedPose(AutoPaths::SwervePose bluePose);
  static std::vector<AutoPaths::SwervePose> GetRedPoses(std::vector<AutoPaths::SwervePose> bluePoses);
  static FieldConstants::ScorePair GetScoringPos(int pos, int height, bool red);
  static bool IsCone(int pos);
  static int GetExpectedTagId(int pos, bool red);
};