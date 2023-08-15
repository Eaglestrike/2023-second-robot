#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector; //!< Alias to vector namespace

/**
 * Interface with an individual swerve module
*/
class SwerveModule {
public:
  SwerveModule(int driveMotorId, int angleMotorId, int encoderId, double kP, double kI, double kD, bool driveInverted, bool encoderInverted, bool angMotorInverted, double offset);

  double GetEncoderReading();
  vec::Vector2D GetVelocity();

  void SetVector(vec::Vector2D vec);
  void SetSpeed(double speed);
  void SetAngle(double angle);
  void SetPID(double kP, double kI, double kD);

  void Periodic();

private:
  bool ShouldFlip(vec::Vector2D curAng, vec::Vector2D targetAng);

  WPI_TalonFX m_driveMotor;
  WPI_TalonFX m_angleMotor;
  WPI_CANCoder m_encoder;
  frc2::PIDController m_controller;

  bool m_flipped;
  bool m_driveInverted; // if the drive motor is inverted (at angle = 0, positive voltage = negative movement)
  bool m_encoderInverted; // if positive encoder values = clockwise when robot is upright
  bool m_angMotorInverted; // if positive angle motor voltage = clockwise when robot is upright
  vec::Vector2D m_targetAngle;
  double m_targetSpeed;
  double m_offset;
};
