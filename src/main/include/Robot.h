// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <memory>
#include <string>

#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Controller/Controller.h"
#include "Drive/SwerveControl.h"
#include "Intake/Intake.h"
#include "Drive/SwerveModule.h"
#include "Util/thirdparty/simplevectors.hpp"
#include "Elevator/Elevator.h"

namespace vec = svector;

class Robot : public frc::TimedRobot {
 public:
  Robot();

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;


  // elevator
  Elevator elevator_;
 private:
  // IMU acclerometer and gyroscope
  // Gives information on orientation and acceleration
  std::shared_ptr<AHRS> m_navx;

  // swerve
  SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  std::shared_ptr<SwerveControl> m_swerveController;

  //Controller
  Controller m_controller;

  Intake m_intake;

  // temp odometry
  vec::Vector2D m_pos;
};
