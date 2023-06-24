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
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "Utils.h"

Robot::Robot()
    : m_lJoy{0}, m_rJoy{1},
      m_swerveFr{SwerveConstants::FR_DRIVE_ID, SwerveConstants::FR_TURN_ID, SwerveConstants::FR_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::FR_INVERTED, SwerveConstants::FR_OFFSET},
      m_swerveBr{SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_TURN_ID, SwerveConstants::BR_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::BR_INVERTED, SwerveConstants::BR_OFFSET},
      m_swerveFl{SwerveConstants::FL_DRIVE_ID, SwerveConstants::FL_TURN_ID, SwerveConstants::FL_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::FL_INVERTED, SwerveConstants::FL_OFFSET},
      m_swerveBl{SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_TURN_ID, SwerveConstants::BL_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::BL_INVERTED, SwerveConstants::BL_OFFSET},
      m_rFr{SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE},
      m_rBr{-SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE},
      m_rFl{SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE},
      m_rBl{-SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE},
      m_odometry{m_swerveController, m_navx}
{
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  std::array<vec::Vector2D, 4> radiiArray{{m_rFr, m_rBr, m_rFl, m_rBl}};
  m_swerveController = std::make_shared<SwerveControl>(moduleArray, radiiArray, 0, 1, 0);

  // AddPeriodic([&](){
  //   // ODOMETRY
  //   vec::Vector2D pos = m_odometry.GetPosition();
  //   double ang = m_odometry.GetAng();

  //   frc::SmartDashboard::PutString("KF pos", pos.toString());
  //   frc::SmartDashboard::PutNumber("KF ang", ang);

  //   m_odometry.Periodic();
  //   // END ODOMETRY
  // }, 5_ms, 2_ms);

  // navx
  try
  {
    m_navx = std::make_shared<AHRS>(frc::SerialPort::kUSB);
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


  // set PID for wheels and angle correction
  m_swerveController->SetAngleCorrectionPID(SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D);

  frc::SmartDashboard::PutNumber("KF E0", OdometryConstants::E0);
  frc::SmartDashboard::PutNumber("KF Q", OdometryConstants::Q);
  frc::SmartDashboard::PutNumber("KF kAng", OdometryConstants::CAM_TRUST_KANG);
  frc::SmartDashboard::PutNumber("KF kPos", OdometryConstants::CAM_TRUST_KPOS);
  frc::SmartDashboard::PutNumber("KF maxtime", OdometryConstants::MAX_TIME);

  m_navx->ZeroYaw();
  m_swerveController->ResetAngleCorrection();

  // TEMPORARY, REMOVE LATER!!!
  // pretend at AprilTag 7
  m_odometry.SetStart({1.8669, 2.748026}, M_PI);
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
  if (m_lJoy.GetTrigger())
  {
    // double kP = frc::SmartDashboard::GetNumber("wheel kP", SwerveConstants::TURN_P);
    // double kI = frc::SmartDashboard::GetNumber("wheel kI", SwerveConstants::TURN_I);
    // double kD = frc::SmartDashboard::GetNumber("wheel kD", SwerveConstants::TURN_D);

    // m_swerveFl.SetPID(kP, kI, kD);
    // m_swerveFr.SetPID(kP, kI, kD);
    // m_swerveBl.SetPID(kP, kI, kD);
    // m_swerveBr.SetPID(kP, kI, kD);

    // double kP2 = frc::SmartDashboard::GetNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
    // double kI2 = frc::SmartDashboard::GetNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
    // double kD2 = frc::SmartDashboard::GetNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);
    // m_swerveController->SetAngleCorrectionPID(kP2, kI2, kD2);

    double E0 = frc::SmartDashboard::GetNumber("KF E0", OdometryConstants::E0);
    double Q = frc::SmartDashboard::GetNumber("KF Q", OdometryConstants::Q);
    double kAng = frc::SmartDashboard::GetNumber("KF kAng", OdometryConstants::CAM_TRUST_KANG);
    double kPos = frc::SmartDashboard::GetNumber("KF kPos", OdometryConstants::CAM_TRUST_KPOS);
    double maxTime = frc::SmartDashboard::GetNumber("KF maxtime", OdometryConstants::MAX_TIME);

    m_odometry.SetKFTerms(E0, Q, kAng, kPos, maxTime);
  }

  if (m_rJoy.GetTrigger())
  {
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection();
    m_pos = {0, 0};
    m_odometry.Reset();
  }

  // frc::SmartDashboard::PutNumber("fl encoder", m_swerveFl.GetEncoderReading());
  // frc::SmartDashboard::PutNumber("fr encoder", m_swerveFr.GetEncoderReading());
  // frc::SmartDashboard::PutNumber("bl encoder", m_swerveBl.GetEncoderReading());
  // frc::SmartDashboard::PutNumber("br encoder", m_swerveBr.GetEncoderReading());

  // frc::SmartDashboard::PutString("fl velocity", m_swerveFl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("fr velocity", m_swerveFr.GetVelocity().toString());
  // frc::SmartDashboard::PutString("bl velocity", m_swerveBl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("br velocity", m_swerveBr.GetVelocity().toString());
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
  // SWERVE DRIVE
  double lx = m_lJoy.GetRawAxis(0);
  double ly = -m_lJoy.GetRawAxis(1);

  double rx = m_rJoy.GetRawAxis(0);

  double vx = std::clamp(ly, -1.0, 1.0) * 12.0;
  double vy = std::clamp(lx, -1.0, 1.0) * 12.0;
  double w = -std::clamp(rx, -1.0, 1.0) * 12.0;

  // dead zones
  if (std::abs(lx) < 0.1 && std::abs(ly) < 0.1) {
    vx = 0;
    vy = 0;
  }
  if (std::abs(rx) < 0.1) {
    w = 0;
  }

  double curYaw = m_navx->GetYaw();
  curYaw = Utils::DegToRad(curYaw);

  vec::Vector2D setVel = {vx, -vy};
  m_swerveController->SetRobotVelocity(setVel, w, curYaw, 0.005);
  m_swerveController->Periodic();
  // END SWERVE DRIVE

  // TEMP, DELETE LATER
  // vec::Vector2D pos = m_odometry.GetPosition();
  // double ang = m_odometry.GetAng();

  // frc::SmartDashboard::PutString("KF pos", pos.toString());
  // frc::SmartDashboard::PutNumber("KF ang", ang);

  m_odometry.Periodic();
  // END DELETE LATER
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
