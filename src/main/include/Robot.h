// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <vector>
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
#include "Auto/SwerveAutoPath.h"
#include "Auto/AutoStage.h"
#include "Auto/EIAutoPath.h"
#include "Drive/Odometry.h"
#include "Drive/SwerveControl.h"
#include "Drive/SwerveModule.h"
#include "Util/SocketClient.h"
#include "Util/thirdparty/simplevectors.hpp"

#include "Auto/SwerveAutoPath.h"
#include "Auto/AutoConstants.h"

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
  void initEIAutoDBG();
  void periodicEIAutoDBG();
  void initAutoStagesDBG();
  void periodicAutoStagesDBG();
  // timer
  double m_prevTime;

  // smartdashboard
  frc::SendableChooser<std::string> m_startPosChooser;
  frc::SendableChooser<std::string> m_EIautochooser;

  frc::Field2d m_field;

  // IMU acclerometer and gyroscope
  // Gives information on orientation and acceleration
  std::shared_ptr<AHRS> m_navx;

  // swerve
  SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  SwerveControl *m_swerveController;

  // odometry
  vec::Vector2D m_startPos; // offset; starting position on field relative to apriltag origin, can use for trim
  double m_startAng; // offset; starting angle (radians) on field relative to +x axis of apriltag coords, can use for trim
  double m_joystickAng;
  Odometry m_odometry;

  //Controller
  Controller m_controller;

  // A8uto
  AutoLineup m_autoLineup;
  //AutoManager m_auto_manager;
  ElevatorIntake m_EI{};
  Rollers m_rollers{};
  LidarReader m_lidar;
  EIAutoPath m_eiAutoPath{ElevatorIntake::TargetState::STOWED, false};
  std::vector<AutoPathInit> m_initpaths;
  AutoStage m_testStage;
  // AutoManager m_auto_manager;
  // SwerveAutoPath m_swerve_auto{*m_swerveController, SwerveAutoConstants::swerve_poses};

  // TEMP, for testing
  double m_curVolts;
  double m_prevTimeTest;
  wpi::log::DoubleLogEntry m_speedLog;
  wpi::log::DoubleLogEntry m_voltsLog;
  // END TEMP

  // jetson
  SocketClient m_client;
  // Intake m_intake;
  bool m_red;
  int m_posVal; // for auto lineup socring positions
  int m_heightVal;
};
