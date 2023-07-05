#include "Odometry.h"

#include <cmath>
#include <iostream>

#include "Constants.h"
#include "Utils.h"

/**
 * Constructor
 * 
 * Initializes starting position and angle to 0, and this can be changed later with SetStart() 
 * 
 * @param posOffset position offset in robot cpp
 * @param angOffset angle offset in robot cpp
 */
Odometry::Odometry(vec::Vector2D *posOffset, double *angOffset)
  : m_filter{OdometryConstants::E0, OdometryConstants::Q, OdometryConstants::CAM_TRUST_KANG, OdometryConstants::CAM_TRUST_KPOS, OdometryConstants::CAM_TRUST_KPOSINT, OdometryConstants::MAX_TIME,
  posOffset, angOffset}, m_prevId{-1} {}

/**
 * Sets Kalman filter terms
 * 
 * Use this for debugging
 * 
 * It is recommended to reset odometry after setting terms
 * 
 * All of these must be > 0 or things will break
 * 
 * @param E0 initial error covariance
 * @param Q noise of wheels
 * @param kAng angle constant in logistic function higher = lower trust in camera for higher velocities
 * @param k constant of proportionality between speed and camera noise
 * @param kPosInt constant of camera noise when robot not moving
 * @param maxTime max time before ignore, in s
*/
void Odometry::SetKFTerms(double E0, double Q, double kAng, double k, double kPosInt, double maxTime) {
  m_filter.SetTerms(E0, Q, kAng, k, kPosInt, maxTime);
}

/**
 * Applies corrections from camera data
 * 
 * @param camPos Position data from camera
 * @param camAng Angle from camera (or navX, whichever is better), in degrees
 * @param tagID Apriltag ID
 * @param age delay measurement from camera (combined delay from camera to jetson and from jetson to rio through network)
 * @param uniqueId unique ID from camera
*/
void Odometry::SetCamData(vec::Vector2D camPos, double camAng, std::size_t tagID, std::size_t age, std::size_t uniqueId)
{
  double angNavX = GetAng();
  vec::Vector2D vecRot = rotate(camPos, Utils::DegToRad(angNavX - 90));
  vec::Vector2D tagPos;

  // check that ID is actually unique
  if (static_cast<long long>(uniqueId) == m_prevId) {
    return;
  }
  m_prevId = static_cast<long long>(uniqueId);

  // I know I can use an array, i was just being an idiot when writing this
  switch (tagID) {
    case 1:
      // std::cout << "tag1: ";
      tagPos = FieldConstants::TAG1;
      break;
    case 2:
      // std::cout << "tag2: ";
      tagPos = FieldConstants::TAG2;
      break;
    case 3:
      // std::cout << "tag3: ";
      tagPos = FieldConstants::TAG3;
      break;
    case 4:
      // std::cout << "tag4: ";
      tagPos = FieldConstants::TAG4;
      break;
    case 5:
      // std::cout << "tag5: ";
      tagPos = FieldConstants::TAG5;
      break;
    case 6:
      // std::cout << "tag6: ";
      tagPos = FieldConstants::TAG6;
      break;
    case 7:
      // std::cout << "tag7: ";
      tagPos = FieldConstants::TAG7;
      break;
    case 8:
      // std::cout << "tag8: ";
      tagPos = FieldConstants::TAG8;
      break;
    default:
      // std::cout << "bad detect" << std::endl;
      return; // unrecognized tag; don't process
  }

  vec::Vector2D robotPos = tagPos - vecRot;

  // std::cout << robotPos.toString() << std::endl;

  // not using camAng, because it relies on existing odometry measurements to get accurate and ideally it's its own, independent measurement
  // @todo figure out if ^^^ is right
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  //         substituting angnavX here vvvvvv becaues of waht's mentioned in comment above
  m_filter.UpdateFromCamera(robotPos, Utils::DegToRad(angNavX), age, curTimeMs);
}

/**
 * Resets current position and angle
 * 
 * Should do this while robot is facing AWAY
*/
void Odometry::Reset() {
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  m_filter.Reset(curTimeMs);
}

/**
 * Gets current position world frame
 * 
 * @param posOffset starting position
 * 
 * @returns current predicted position
*/
vec::Vector2D Odometry::GetPosition(vec::Vector2D posOffset) const {
  return m_filter.GetEstimatedPos() + posOffset;
}

/**
 * Gets current estimated angle world frame
 * 
 * @returns estimated angle, in radians
*/
double Odometry::GetAng() const {
  return m_filter.GetEstimatedAng();
}

/**
 * Periodic function
 * 
 * @param ang angle of robot world frame, radians
 * @param avgVelocity average velocity world frame
*/
void Odometry::Periodic(double ang, vec::Vector2D avgVelocity) {
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  m_filter.PredictFromWheels(avgVelocity, ang, curTimeMs);
}