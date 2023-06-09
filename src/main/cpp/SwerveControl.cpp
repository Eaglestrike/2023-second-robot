#include "SwerveControl.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "Mathutil.h"

/**
 * Constructor
 *
 * @param modules An std::array of references to 4 swerve modules
 * @param radii An std::array of 4 vec::Vector2D objects, each one representing a vector originating
 * from the center of the robot and pointing to the corresponding swerve module index in the modules parameter
 * @param kS kS value for feedforward
 * @param kV kV value for feedforward
 * @param kA kA value for feedforward
 *
 * @note feedforward relates speed to voltage
 */
SwerveControl::SwerveControl(RefArray<SwerveModule> modules, std::array<vec::Vector2D, 4> radii, double kS, double kV, double kA)
    : m_modules{modules}, m_radii{radii}, m_kS{kS}, m_kV{kV}, m_kA{kA}, m_curAngle{0}, m_angleCorrector{0.1, 0, 0.01}
{
  m_angleCorrector.EnableContinuousInput(-M_PI, M_PI);
  ResetFeedForward();
}

/**
 * Gets robot velocity by averaging the velocities of the modules
 *
 * @note Assumes modules have the same mass
 * 
 * @param ang NavX angle
 *
 * @returns Robot velocity
 */
vec::Vector2D SwerveControl::GetRobotVelocity(double ang)
{
  std::vector<vec::Vector2D> vectors;
  for (auto module : m_modules)
  {
    vectors.push_back(module.get().GetVelocity());
  }

  auto avg = Mathutil::GetVecAverage(vectors);
  return vec::rotate(avg, ang); // rotate by navx ang
}

/**
 * Resets previous speeds for acceleration feed forward
 */
void SwerveControl::ResetFeedForward()
{
  m_prevSpeeds.fill(0);
}

/**
 * Resets angle correction angle to zero
 *
 * @note Always call this after calling navx::ZeroYaw()
 */
void SwerveControl::ResetAngleCorrection()
{
  m_curAngle = 0;
}

/**
 * Sets feed foward terms relating speed to voltage
 *
 * @param kS kS term
 * @param kV kV term
 * @param kA kA term
 */
void SwerveControl::SetFeedForward(double kS, double kV, double kA)
{
  m_kS = kS;
  m_kV = kV;
  m_kA = kA;
}

/**
 * Sets Angle Correction PID
 *
 * @param kP kP term
 * @param kI kI term
 * @param kD kD term
 */
void SwerveControl::SetAngleCorrectionPID(double kP, double kI, double kD)
{
  m_angleCorrector.SetPID(kP, kI, kD);
}

/**
 * Sets robot velocity
 *
 * @param vel Velocity to set
 * @param angVel Angular velocity, + is counterclockwise, - is clockwise
 * @param ang Current navX angle, in radians
 * @param time Time between readings
 */
void SwerveControl::SetRobotVelocity(vec::Vector2D vel, double angVel, double ang, double time)
{
  std::vector<vec::Vector2D> vecPrints;
  vecPrints.resize(4);

  frc::SmartDashboard::PutNumber("cjurrent angle", m_curAngle);

  if (!Mathutil::NearZero(angVel))
  {
    // if turning, track current angle
    m_curAngle = ang;
  }

  if (!Mathutil::NearZero(vel) && Mathutil::NearZero(angVel))
  {
    // if not turning, correct robot so that it doesnt turn
    angVel = m_angleCorrector.Calculate(ang, m_curAngle);
    frc::SmartDashboard::PutNumber("pidout", angVel);
    angVel = std::clamp(angVel, -SwerveConstants::MAX_VOLTS, SwerveConstants::MAX_VOLTS);
  }


  for (std::size_t i = 0; i < 4; i++)
  {
    // computes vectors in 3D
    vec::Vector3D vel3D = {x(vel), y(vel), 0};
    vec::Vector3D angVel3D = {0, 0, angVel};
    vec::Vector3D module3D = {x(m_radii[i]), y(m_radii[i]), 0};

    // vector addition for velocity
    vec::Vector3D moduleWorld = rotateGamma(module3D, ang);        // rotates radius vector to world frame
    vec::Vector3D velWorld = vel3D + cross(angVel3D, moduleWorld); // linear + tangential velocity for module

    // rotates velocity vector to body frame
    vec::Vector3D velBody3D = rotateGamma(velWorld, -ang);

    // sets module velocity
    vec::Vector2D velBody = {x(velBody3D), y(velBody3D)};

    // speed from ff calculations, then resize velBody to match ff calculations
    double speed = m_kS + m_kV * magn(velBody) + m_kA * (magn(velBody) - m_prevSpeeds[i]) / time;
    if (!Mathutil::NearZero(velBody) && !Mathutil::NearZero(speed))
    {
      velBody = normalize(velBody) * speed;
    }
    m_prevSpeeds[i] = magn(velBody);

    // set vector
    vecPrints[i] = velBody;
    m_modules[i].get().SetVector(velBody);
  }
}

/**
 * Periodic function
 *
 * @note to self: CALL ME!!!! cALLL ME!!!!! you blITHERignnGG IDIOT!
 */
void SwerveControl::Periodic()
{
  for (auto module : m_modules)
  {
    module.get().Periodic();
  }
}
