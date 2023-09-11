// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <memory>
#include <string>

#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Controller/Controller.h"
#include "Drive/AutoDrive.h"
#include "Drive/Odometry.h"
#include "Drive/SwerveControl.h"
#include "Drive/SwerveModule.h"
#include "Util/SocketClient.h"
#include "Util/thirdparty/simplevectors.hpp"

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
  // smartdashboard
  frc::SendableChooser<std::string> m_startPosChooser;
  frc::Field2d m_field;

  // IMU acclerometer and gyroscope
  // Gives information on orientation and acceleration
  std::shared_ptr<AHRS> m_navx;

  // swerve
  SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  vec::Vector2D m_rFr, m_rBr, m_rFl, m_rBl;
  SwerveControl *m_swerveController;

  // odometry
  vec::Vector2D m_startPos; // offset; starting position on field relative to apriltag origin, can use for trim
  double m_startAng; // offset; starting angle (radians) on field relative to +x axis of apriltag coords, can use for trim
  double m_joystickAng;
  Odometry m_odometry;

  //Controller
  Controller m_controller;

  // auto
  AutoDrive m_autoDrive;

  // jetson
  SocketClient m_client;
};
