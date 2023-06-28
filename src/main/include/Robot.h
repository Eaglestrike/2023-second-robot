// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <memory>
#include <string>

#include <AHRS.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Odometry.h"
#include "SocketClient.h"
#include "SwerveControl.h"
#include "SwerveModule.h"
#include "thirdparty/simplevectors.hpp"

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

  // inputs
  frc::Joystick m_lJoy;
  frc::Joystick m_rJoy;

  // IMU
  AHRS *m_navx;

  // swerve
  SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  vec::Vector2D m_rFr, m_rBr, m_rFl, m_rBl;
  SwerveControl *m_swerveController;

  // odometry
  vec::Vector2D m_startPos; // offset; starting position on field relative to apriltag origin, can use for trim
  double m_startAng; // offset; starting angle (radians) on field relative to +x axis of apriltag coords, can use for trim
  double m_joystickAng;
  Odometry m_odometry;

  // jetson
  SocketClient m_client;
};
