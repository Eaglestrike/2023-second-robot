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
      m_swerveFr(SwerveConstants::FR_CONFIG, true, true),
      m_swerveBr(SwerveConstants::BR_CONFIG, true, true),
      m_swerveFl(SwerveConstants::FL_CONFIG, true, true),
      m_swerveBl(SwerveConstants::BL_CONFIG, true, true)
{
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  m_swerveController = std::make_shared<SwerveControl>(moduleArray, true, false);

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

void Robot::RobotInit(){
  m_navx->ZeroYaw();

  m_swerveController->Init();
  m_swerveController->SetFeedForward(0.0 , 1.0, 0.0);
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
    m_swerveFl.UpdateShuffleboard();
    m_swerveFr.UpdateShuffleboard();
    m_swerveBl.UpdateShuffleboard();
    m_swerveBr.UpdateShuffleboard();
  }

  if (m_controller.getPressed(ZERO_YAW))
  {
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection();
    m_pos = {0, 0};
  }
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
void Robot::AutonomousInit(){
}

void Robot::AutonomousPeriodic(){
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  m_intake.TeleopPeriodic();
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
