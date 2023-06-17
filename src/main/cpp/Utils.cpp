#include "Utils.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

#include <frc/Timer.h>

/**
 * Gets the value with the minimum absolute value between two numbers
 *
 * @param a One number
 * @param b Another number
 *
 * @returns a or b, depending on which one has a lesser absolute value
 */
double Utils::AbsMin(const double a, const double b)
{
  return std::abs(a) < std::abs(b) ? a : b;
}

/**
 * Averages a std::vector of vec::Vector2D
 *
 * @param vectors the non-zero sized array of vectors
 *
 * @note If the size of vectors is 0, then returns a zero vector.
 *
 * @returns The averaged vector
 */
vec::Vector2D Utils::GetVecAverage(const std::vector<vec::Vector2D> vectors)
{
  vec::Vector2D res;
  if (vectors.size() == 0)
  {
    return res;
  }

  for (auto vec : vectors)
  {
    res += vec;
  }

  res /= vectors.size();

  return res;
}

/**
 * Determines if a number is near zero
 *
 * @param num Number
 * @param tolerance Tolerance for being near zero
 */
bool Utils::NearZero(const double num, const double tolerance)
{
  return std::abs(num) <= tolerance;
}

/**
 * Determines if a vector is near zero
 *
 * @param vec Vector
 * @param tolerance Tolerance for being near zero
 */
bool Utils::NearZero(const vec::Vector2D vec, const double tolerance)
{
  for (auto component : vec)
  {
    if (!NearZero(component, tolerance))
    {
      return false;
    }
  }
  return true;
}

/**
 * Gets the current time from robot start in ms
 * 
 * @returns Current robot time in ms
*/
std::size_t Utils::GetCurTimeMs() {
  auto curTime = frc::Timer::GetFPGATimestamp();
  auto timeMs = curTime.convert<units::millisecond>();
  return static_cast<std::size_t>(timeMs.value());
}

/**
 * Converst degrees to radians
 * 
 * @param deg Angle in degrees
 * 
 * @returns Angle in radians
*/
double Utils::DegToRad(const double deg) {
  return deg * (M_PI / 180.0);
}

/**
 * Converts radians to degrees
 * 
 * @param rad Angle in radians
 * 
 * @returns Angle in degrees
*/
double Utils::RadToDeg(const double rad) {
  return rad * (180.0 / M_PI);
}