#include "Odometry.h"

#include <cmath>
#include <iostream>

#include "Constants.h"
#include "Utils.h"

/**
 * Constructor
 * 
 * Initializes starting position and angle to 0, and this can be changed later with SetStart() 
 */
Odometry::Odometry()
  : m_startAng{0}, m_curAng{0}, m_filter{
   OdometryConstants::E0, OdometryConstants::Q, OdometryConstants::CAM_TRUST_KANG, OdometryConstants::CAM_TRUST_KPOS, OdometryConstants::MAX_TIME
  } {}

/**
 * Sets swerve controller and navX pointers
 * 
 * @param swerveController pointer to swerve controller
 * @param navx Pointer to navx
*/
void Odometry::SetPointers(SwerveControl *swerveController, AHRS *navx) {
  m_swerveController = swerveController;
  m_navx = navx;
}

/**
 * Sets start position
 * 
 * This can be used for trimming
 * 
 * @param startPos starting position
*/
void Odometry::SetStartPos(vec::Vector2D startPos) {
  m_startPos = startPos;
}

/**
 * Sets start angle
 * 
 * This can be used for trimming
 * 
 * @param startAng starting angle, in radians
*/
void Odometry::SetStartAng(double startAng) {
  m_startAng = startAng;
}

/**
 * Sets start position and angle
 * 
 * Call this after setting position in shuffleboard
 * 
 * @param startPos starting position
 * @param startAng starting angle, in radians
*/
void Odometry::SetStart(vec::Vector2D startPos, double startAng) {
  m_startPos = startPos;
  m_startAng = startAng;
}

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
 * @param maxTime max time before ignore, in s
*/
void Odometry::SetKFTerms(double E0, double Q, double kAng, double k, double maxTime) {
  m_filter.SetTerms(E0, Q, kAng, k, maxTime);
}

/**
 * Applies corrections from camera data
 * 
 * @todo Implement
 * 
 * @param camPos Position data from camera
 * @param camAng Angle from camera (or navX, whichever is better), in degrees
 * @param angNavX navX angle, in degrees
 * @param camID ID of camera
 * @param howLongAgo delay measurement from camera (combined delay from camera to jetson and from jetson to rio through network)
*/
void Odometry::SetCamData(vec::Vector2D camPos, double camAng, double angNavX, std::size_t camID, std::size_t howLongAgo)
{
  vec::Vector2D vecRot = rotate(camPos, Utils::DegToRad(angNavX - 90));
  vec::Vector2D tagPos;

  switch (camID) {
    case 1:
      tagPos = FieldConstants::TAG1;
      break;
    case 2:
      tagPos = FieldConstants::TAG2;
      break;
    case 3:
      tagPos = FieldConstants::TAG3;
      break;
    case 4:
      tagPos = FieldConstants::TAG4;
      break;
    case 5:
      tagPos = FieldConstants::TAG5;
      break;
    case 6:
      tagPos = FieldConstants::TAG6;
      break;
    case 7:
      tagPos = FieldConstants::TAG7;
      break;
    case 8:
      tagPos = FieldConstants::TAG8;
      break;
    default:
      return; // unrecognized tag; don't process
  }

  vec::Vector2D robotPos = tagPos - vecRot;

  // not using camAng, because it relies on existing odometry measurements to get accurate and ideally it's its own, independent measurement
  // @todo figure out if ^^^ is right
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  //         substituting angnavX here vvvvvv becaues of waht's mentioned in comment above
  m_filter.UpdateFromCamera(robotPos, Utils::DegToRad(angNavX), howLongAgo, curTimeMs);
}

/**
 * Resets current position and angle
*/
void Odometry::Reset() {
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  m_filter.Reset(curTimeMs);
}

/**
 * Gets current position
 * 
 * @returns current predicted position
*/
vec::Vector2D Odometry::GetPosition() const {
  return m_filter.GetEstimatedPos();
}

/**
 * Gets current estimated angle
 * 
 * @returns estimated angle, in radians
*/
double Odometry::GetAng() const {
  return m_filter.GetEstimatedAng();
}

/**
 * Periodic function, call this every periodic!!!1111@!!!!
 * 
 * dont forgor
*/
void Odometry::Periodic() {
  double ang = m_navx->GetYaw() + Utils::RadToDeg(m_startAng);
  vec::Vector2D avgVelocityWorld = m_swerveController->GetRobotVelocity(Utils::DegToRad(ang));
  std::size_t curTimeMs = Utils::GetCurTimeMs();
  m_filter.PredictFromWheels(avgVelocityWorld, Utils::DegToRad(ang), curTimeMs);
}