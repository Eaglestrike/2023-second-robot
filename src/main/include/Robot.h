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
#include "Elevator/ElevatorIntake.h"
#include "Drive/SwerveModule.h"
#include "Util/thirdparty/simplevectors.hpp"
#include "Util/Mathutil.h"

#include "Elevator/Lidar/LidarReader.h"

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

 private:
  //Controller
  Controller m_controller;

  ElevatorIntake m_elevatorIntake;

  // IMU acclerometer and gyroscope
  // Gives information on orientation and acceleration
  std::shared_ptr<AHRS> m_navx;

  // swerve
  double m_prevTime;
  SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  SwerveControl *m_swerveController;
};
