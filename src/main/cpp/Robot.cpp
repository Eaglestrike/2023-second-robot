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

Robot::Robot(){
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
  if (m_controller.getPressed(ELEVATOR_UPDATE)) {
    // elevator_.UpdateShuffleboard();
  }

  if (m_controller.getPressed(ELEVATOR_EXTEND_STOWED)) {
    // elevator_.Stow();
  }
  else if (m_controller.getPressed(ELEVATOR_EXTEND_LOW)) {
    // elevator_.ExtendLow();
  }
  else if (m_controller.getPressed(ELEVATOR_EXTEND_MID)) {
    // elevator_.ExtendMid();
  }
  else if (m_controller.getPressed(ELEVATOR_EXTEND_HIGH)) {
    // elevator_.ExtendHigh();
  }
  else{
    // elevator_.HoldPosition();
  }

  if (m_controller.getRawAxis(ELEVATOR_SET_MANUAL) > 0.75) {
    // elevator_.setManualVolts(m_controller.getRawAxis(ELEVATOR_RANGE));
  }

  // elevator_.Periodic();
  m_elevatorIntake.TeleopPeriodic();

  // TODO: check this and the corresponding mapping in ControllerMap.h
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
  if (m_controller.getPressed(MANUAL1) && m_controller.getPressed(MANUAL2)) {
    // double intakeX = m_controller.getWithDeadContinuous(MOVE_INTAKE, 0.1);
    // intakeX = intakeX * IntakeConstants::WRIST_MAX_VOLTS;
    // m_intake.ManualPeriodic(intakeX);
  } else {
    // m_intake.TeleopPeriodic();
    m_elevatorIntake.TeleopPeriodic();
  }
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
