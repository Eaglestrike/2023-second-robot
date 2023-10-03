#include "Drive/SwerveModule.h"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

#include "Drive/DriveConstants.h"
#include "Util/Mathutil.h"

#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Constructor
 *
 * @param config swerve config
 * @param enabled enable mechanism
 * @param shuffleboard enable shuffleboard prints
 */
SwerveModule::SwerveModule(SwerveConstants::SwerveConfig config, bool enabled, bool shuffleboard):
      Mechanism(config.name, enabled, shuffleboard),
      m_driveMotor{config.driveMotorId, "Drivebase"},
      m_angleMotor{config.angleMotorId, "Drivebase"},
      m_encoder{config.encoderId, "Drivebase"},
      m_controller{config.kP, config.kI, config.kD},
      m_flipped{false},
      m_driveInverted{config.driveInverted},
      m_encoderInverted{config.encoderInverted},
      m_angMotorInverted{config.angMotorInverted},
      m_targetSpeed{0},
      m_offset{config.offset},
      m_position{config.position}
{
  m_encoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
  // m_encoder.ConfigMagnetOffset(offset);
  m_controller.EnableContinuousInput(-M_PI, M_PI);
}

/**
 * Gets velocity in m/s
 *
 * @returns velocity in m/s
 */
vec::Vector2D SwerveModule::GetVelocity()
{
  //                                   (x ticks / 1 100ms) * (10 100ms / 1 s) * (2π motor radians / TALON_FX_COUNTS_PER_REV ticks) * (1 wheel radian / WHEEL_GEAR_RATIO motor radians) * (WHEEL_RADIUS m / 1 wheel radian)
  double curMotorSpeed = m_driveMotor.GetSelectedSensorVelocity() * 10.0 * (2.0 * M_PI / SwerveConstants::TALON_FX_COUNTS_PER_REV) * (1 / SwerveConstants::WHEEL_GEAR_RATIO) * SwerveConstants::WHEEL_RADIUS;
  double curAng = GetCorrectedEncoderReading() * (M_PI / 180);

  auto resVec = vec::Vector2D{std::cos(curAng), std::sin(curAng)} * curMotorSpeed;

  return m_driveInverted ? -resVec : resVec;
}

/**
 * Gets position in m to center of robot
 *
 * @returns position in m
 */
vec::Vector2D SwerveModule::getPosition(){
  return m_position;
}

/**
 * Gets corrected angle encoder reading, after applying offset and encoder inversion
 *
 * @returns angle encoder reading, in degrees
 */
double SwerveModule::GetCorrectedEncoderReading()
{
  double val = m_encoder.GetAbsolutePosition() - m_offset;

  if (m_encoderInverted) {
    val = -val;
  }

  return val;
}

/**
 * Gets raw angle encoder reading
 * 
 * @returns Raw encoder reading
 */
double SwerveModule::GetRawEncoderReading() {
  double val = m_encoder.GetAbsolutePosition();
  return val;
}

/**
 * Sets velocity
 *
 * @param vec Velocity vector
 *
 * @note If vector is 0, then sets angle to 0
 */
void SwerveModule::SetVector(vec::Vector2D vec)
{
  m_targetSpeed = vec::magn(vec);

  if (!Utils::NearZero(vec))
  {
    m_targetAngle = vec::normalize(vec);
  }
  else
  {
    m_targetAngle = vec::Vector2D{1, 0};
  }
}

/**
 * Sets speed
 *
 * @param speed speed to set
 */
void SwerveModule::SetSpeed(double speed)
{
  m_targetSpeed = speed;
}

/**
 * Sets angle
 *
 * @param angle Angle to set, in degrees
 */
void SwerveModule::SetAngle(double angle)
{
  angle = angle * (M_PI / 180);
  m_targetAngle = {std::cos(angle), std::sin(angle)};
}

/**
 * Sets pid
 *
 * @param kP P-value
 * @param kI I-value
 * @param kD D-value
 */
void SwerveModule::SetPID(double kP, double kI, double kD)
{
  m_controller.SetPID(kP, kI, kD);
}

void SwerveModule::CoreTeleopPeriodic()
{
  // get current angle
  double correctedEncoderReading = GetCorrectedEncoderReading();
  double ang = correctedEncoderReading * (M_PI / 180.0);
  vec::Vector2D angVec = {std::cos(ang), std::sin(ang)};

  // flip angle if currently flipped
  if (m_flipped)
  {
    angVec = -angVec;
  }

  // check if module should be flipped
  if (ShouldFlip(angVec, m_targetAngle))
  {
    m_flipped = !m_flipped;
    angVec = -angVec;
    m_controller.Reset(); // Reset because integral and derivative terms will behave wonky
  }

  // calculates PID from error
  double angleOutput = m_controller.Calculate(angVec.angle(), m_targetAngle.angle());
  angleOutput = std::clamp(angleOutput, -SwerveConstants::MAX_VOLTS, SwerveConstants::MAX_VOLTS);

  // make sure positive angle motor voltage = CCW
  if (m_angMotorInverted) {
    angleOutput = -angleOutput;
  }

  // speed calculations
  double speed = 0;
  if (m_flipped)
  {
    speed = m_driveInverted ? m_targetSpeed : -m_targetSpeed;
  }
  else
  {
    speed = m_driveInverted ? -m_targetSpeed : m_targetSpeed;
  }
  speed = std::clamp(speed, -SwerveConstants::MAX_VOLTS, SwerveConstants::MAX_VOLTS);

  // set voltages to motor
  m_driveMotor.SetVoltage(units::volt_t{speed});
  m_angleMotor.SetVoltage(units::volt_t{angleOutput});
}

/**
 * Given current angle and target angle, determines
 *
 * @param curVec current angle, vector form
 * @param targetVec target angle, vector form
 *
 * @returns Whether should flip
 */
bool SwerveModule::ShouldFlip(vec::Vector2D curVec, vec::Vector2D targetVec)
{
  vec::Vector2D curNeg = -curVec;

  // positive angle
  double angle1 = std::acos(std::clamp(
      dot(curVec, targetVec) / (magn(curVec) * magn(targetVec)), -1.0, 1.0));

  // negative angle
  double angle2 = std::acos(std::clamp(
      dot(curNeg, targetVec) / (magn(curNeg) * magn(targetVec)), -1.0, 1.0));

  return angle2 < angle1;
}

void SwerveModule::CoreShuffleboardPeriodic(){
  frc::SmartDashboard::PutNumber(name_ + " encoder", GetRawEncoderReading());

  frc::SmartDashboard::PutString(name_ + " velocity", GetVelocity().toString());
}