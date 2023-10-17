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

Robot::Robot() : 
  m_prevTime{0},
  m_swerveFr{SwerveConstants::FR_CONFIG, true, false},
  m_swerveBr{SwerveConstants::BR_CONFIG, true, false},
  m_swerveFl{SwerveConstants::FL_CONFIG, true, false},
  m_swerveBl{SwerveConstants::BL_CONFIG, true, false}
{
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  m_swerveController = new SwerveControl(moduleArray, true, false);

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
  // if (m_controller.getPressed(ELEVATOR_UPDATE)) {
  //   // elevator_.UpdateShuffleboard();
  // }

  // if (m_controller.getPressed(ELEVATOR_EXTEND_STOWED)) {
  //   // elevator_.Stow();
  // }
  // else if (m_controller.getPressed(ELEVATOR_EXTEND_LOW)) {
  //   // elevator_.ExtendLow();
  // }
  // else if (m_controller.getPressed(ELEVATOR_EXTEND_MID)) {
  //   // elevator_.ExtendMid();
  // }
  // else if (m_controller.getPressed(ELEVATOR_EXTEND_HIGH)) {
  //   // elevator_.ExtendHigh();
  // }
  // else{
  //   // elevator_.HoldPosition();
  // }

  // if (m_controller.getRawAxis(ELEVATOR_SET_MANUAL) > 0.75) {
  //   // elevator_.setManualVolts(m_controller.getRawAxis(ELEVATOR_RANGE));
  // }

  if (m_controller.getPressedOnce(ZERO_YAW))
  {
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection();
  }

  m_elevatorIntake.Periodic();
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

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;
  m_prevTime = curTime;

  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.1);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.1);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.1);

  double vx = std::clamp(lx, -1.0, 1.0) * 12.0;
  double vy = std::clamp(ly, -1.0, 1.0) * 12.0;
  double w = -std::clamp(rx, -1.0, 1.0) * 12.0;

  double curYaw = Utils::DegToRad(m_navx->GetYaw());

  vec::Vector2D setVel = {-vy, -vx};

  m_swerveController->SetFeedForward(0, 1, 0);
  m_swerveController->SetRobotVelocity(setVel, w, curYaw, deltaT);

  m_swerveController->Periodic();

  m_elevatorIntake.TeleopPeriodic();
  bool cone = m_controller.getPressed(CONE);
  if(m_controller.getPressed(SCORE_HIGH))
    m_elevatorIntake.ScoreHigh(cone);
  else if (m_controller.getPressed(SCORE_MID))
    m_elevatorIntake.ScoreMid(cone);
  else if (m_controller.getPressed(SCORE_LOW))
    m_elevatorIntake.ScoreLow(cone);
  else if (m_controller.getPressed(STOW))
    m_elevatorIntake.Stow();
  else if (m_controller.getPressed(HP))
    m_elevatorIntake.IntakeFromHPS(cone);
  else if (m_controller.getRawAxis(GROUND) > 0.75)
    m_elevatorIntake.IntakeFromGround(cone);
  else if (m_controller.getPressed(INTAKE))
    m_elevatorIntake.ToggleRoller(false);
  else if (m_controller.getPressed(OUTTAKE))
    m_elevatorIntake.ToggleRoller(true);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
}

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
