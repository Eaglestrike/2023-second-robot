// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Drive/DriveConstants.h"
#include "Controller/ControllerMap.h"

using namespace Actions;

Robot::Robot():
      m_swerveFr{SwerveConstants::FR_DRIVE_ID, SwerveConstants::FR_TURN_ID, SwerveConstants::FR_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::FR_DRIVE_INVERTED, SwerveConstants::FR_ENCODER_INVERTED, SwerveConstants::FR_ANG_INVERTED, SwerveConstants::FR_OFFSET},
      m_swerveBr{SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_TURN_ID, SwerveConstants::BR_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::BR_DRIVE_INVERTED, SwerveConstants::BR_ENCODER_INVERTED, SwerveConstants::BR_ANG_INVERTED, SwerveConstants::BR_OFFSET},
      m_swerveFl{SwerveConstants::FL_DRIVE_ID, SwerveConstants::FL_TURN_ID, SwerveConstants::FL_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::FL_DRIVE_INVERTED, SwerveConstants::FL_ENCODER_INVERTED, SwerveConstants::FL_ANG_INVERTED, SwerveConstants::FL_OFFSET},
      m_swerveBl{SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_TURN_ID, SwerveConstants::BL_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::BL_DRIVE_INVERTED, SwerveConstants::BL_ENCODER_INVERTED, SwerveConstants::BL_ANG_INVERTED, SwerveConstants::BL_OFFSET},
      m_rFr{SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE},
      m_rBr{-SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE},
      m_rFl{SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE},
      m_rBl{-SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE}
{
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  std::array<vec::Vector2D, 4> radiiArray{{m_rFr, m_rBr, m_rFl, m_rBl}};
  m_swerveController = std::make_shared<SwerveControl>(moduleArray, radiiArray, 0, 1, 0);

  // navx
  try
  {
    m_navx = std::make_shared<AHRS>(frc::SerialPort::kMXP);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }
}

void Robot::RobotInit()
{
  // frc::SmartDashboard::PutNumber("wheel kP", SwerveConstants::TURN_P);
  // frc::SmartDashboard::PutNumber("wheel kI", SwerveConstants::TURN_I);
  // frc::SmartDashboard::PutNumber("wheel kD", SwerveConstants::TURN_D);
  // frc::SmartDashboard::PutNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  // frc::SmartDashboard::PutNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  // frc::SmartDashboard::PutNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);

  frc::SmartDashboard::PutNumber("elevator ks", 0.2);
  frc::SmartDashboard::PutNumber("elevator ka", 0.2);
  frc::SmartDashboard::PutNumber("elevator kv", 0.2);
  frc::SmartDashboard::PutNumber("elevator kg", 0.2);

  m_navx->ZeroYaw();
  m_swerveController->ResetAngleCorrection();

  elevator_.setMaxDistance(90);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  if (m_controller.getPressed(ZERO_DRIVE_PID))
  {
    // double kP = frc::SmartDashboard::GetNumber("wheel kP", SwerveConstants::TURN_P);
    // double kI = frc::SmartDashboard::GetNumber("wheel kI", SwerveConstants::TURN_I);
    // double kD = frc::SmartDashboard::GetNumber("wheel kD", SwerveConstants::TURN_D);
    // double kP2 = frc::SmartDashboard::GetNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
    // double kI2 = frc::SmartDashboard::GetNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
    // double kD2 = frc::SmartDashboard::GetNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);

    // m_swerveFl.SetPID(kP, kI, kD);
    // m_swerveFr.SetPID(kP, kI, kD);
    // m_swerveBl.SetPID(kP, kI, kD);
    // m_swerveBr.SetPID(kP, kI, kD);

    // m_swerveController->SetAngleCorrectionPID(kP2, kI2, kD2);
  }

  if (m_controller.getPressed(ZERO_YAW))
  {
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection();
    m_pos = {0, 0};
  }

  if (m_controller.getPressed(ZERO_FEEDFORWARD)) {
    elevator_.zero_motors();
  }

  // frc::SmartDashboard::PutNumber("fl raw encoder", m_swerveFl.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("fr raw encoder", m_swerveFr.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("bl raw encoder", m_swerveBl.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("br raw encoder", m_swerveBr.GetRawEncoderReading());

  // frc::SmartDashboard::PutString("fl velocity", m_swerveFl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("fr velocity", m_swerveFr.GetVelocity().toString());
  // frc::SmartDashboard::PutString("bl velocity", m_swerveBl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("br velocity", m_swerveBr.GetVelocity().toString());

  frc::SmartDashboard::PutString("left elevator rotation", std::to_string(elevator_.getLeftRotation()));
  frc::SmartDashboard::PutString("right elevator rotation", std::to_string(elevator_.getRightRotation()));


  double dash_ks = frc::SmartDashboard::GetNumber("elevator ks", ElevatorConstants::KS);
  double dash_kv = frc::SmartDashboard::GetNumber("elevator kv", ElevatorConstants::KV);
  double dash_kg = frc::SmartDashboard::GetNumber("elevator kg", ElevatorConstants::KG);
  double dash_ka = frc::SmartDashboard::GetNumber("elevator ka", ElevatorConstants::KA);


  elevator_.setFeedforwardConstants(dash_ks, dash_kv, dash_kg, dash_ka);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.1);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.1);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.1);

  double vx = std::clamp(lx, -1.0, 1.0) * 12.0;
  double vy = std::clamp(ly, -1.0, 1.0) * 12.0;
  double w = -std::clamp(rx, -1.0, 1.0) * 12.0;

  double curYaw = m_navx->GetYaw();
  curYaw = curYaw * (M_PI / 180);

  // frc::SmartDashboard::PutNumber("curYaw", curYaw);

  vec::Vector2D setVel = {-vy, -vx};
  m_swerveController->SetRobotVelocity(setVel, w, curYaw, 0.02);

  m_swerveController->Periodic();

  // temporary
  auto vel = m_swerveController->GetRobotVelocity(curYaw);
  m_pos += vel * 0.02;

  // frc::SmartDashboard::PutString("pos:", m_pos.toString());
  // frc::SmartDashboard::PutString("vel:", vel.toString());
  // frc::SmartDashboard::PutString("setVel:", setVel.toString());
  // frc::SmartDashboard::PutNumber("setAngVel:", w);

  elevator_.periodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
