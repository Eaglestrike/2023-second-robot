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
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

#include "Controller/Controller.h"

#include "Drive/AutoLineup.h"
#include "Drive/AutoPath.h"
#include "Drive/Odometry.h"
#include "Drive/SwerveControl.h"
#include "Drive/SwerveModule.h"

#include "Elevator/ElevatorIntake.h"
#include "Elevator/Intake/Rollers.h"

#include "Util/SocketClient.h"
#include "Util/thirdparty/simplevectors.hpp"
#include "Util/Mathutil.h"

#include "Elevator/Lidar/LidarReader.h"

#include "ShuffleboardSender/ShuffleboardSender.h"

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
  // timer
  double m_prevTime;

  //Controller
  Controller m_controller;

  //Elevator
  ElevatorIntake m_elevatorIntake;
  LidarReader m_lidar;
  Rollers m_rollers;

  // IMU acclerometer and gyroscope
  // Gives information on orientation and acceleration
  std::shared_ptr<AHRS> m_navx;

  // smartdashboard
  frc::SendableChooser<std::string> m_startPosChooser;
  frc::Field2d m_field;

  // swerve
  SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  SwerveControl *m_swerveController;

  // jetson
  SocketClient m_client;
  bool m_red;
  int m_posVal; // for auto lineup socring positions
  int m_heightVal;

  // odometry
  Odometry m_odometry;
  vec::Vector2D m_startPos; // offset; starting position on field relative to apriltag origin, can use for trim
  double m_startAng; // offset; starting angle (radians) on field relative to +x axis of apriltag coords, can use for trim
  double m_joystickAng;
  bool m_isAutoLineup = false; // UNUSED; disables tag odometry when auto lineup so robot isnt jumpy
  bool m_isTrimming = false; // if true, use ff only
  bool m_isFirstTag = false;

  // auto
  AutoLineup m_autoLineup;
  AutoPath m_autoPath;
  // TEMP, for testing
  // double m_curVolts;
  // double m_prevTimeTest;
  // wpi::log::DoubleLogEntry m_speedLog;
  // wpi::log::DoubleLogEntry m_voltsLog;
  // END TEMP

  ShuffleboardSender m_shuff;
};
