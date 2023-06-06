#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>

#include "thirdparty/simplevectors.hpp"

namespace vec = svector; //!< Alias to vector namespace

/**
 * Interface with an individual swerve module
*/
class SwerveModule {
public:
  SwerveModule(int driveMotorId, int angleMotorId, int encoderId, double kP, double kI, double kD, bool inverted, double offset);

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
  bool m_inverted;
  vec::Vector2D m_targetAngle;
  double m_targetSpeed;
  double m_offset;
};
