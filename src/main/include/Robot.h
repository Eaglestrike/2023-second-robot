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

#include "Auto/AutoDock.h"
#include "Auto/DumbDock.h"
#include "Auto/SadAuto.h"
#include "Auto/ThreePiece.h"
#include "Auto/TwoPieceDock.h"

#include "Controller/Controller.h"

#include "Drive/AutoLineup.h"
#include "Drive/AutoPath.h"
#include "Drive/Odometry.h"
#include "Drive/SwerveControl.h"
#include "Drive/SwerveModule.h"

#include "Elevator/ElevatorIntake.h"
#include "Elevator/Intake/Rollers.h"
#include "Elevator/Lidar/LidarReader.h"

#include "Util/SocketClient.h"
#include "Util/thirdparty/simplevectors.hpp"
#include "Util/Utils.h"

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

  // smartdashboard
  frc::SendableChooser<std::string> m_startPosChooser;
  frc::SendableChooser<std::string> m_autoChooser;
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
  bool m_isAutoLineup = false; // UNUSED; disables tag odometry when auto lineup so robot isnt jumpy
  bool m_isTrimming = false; // if true, use ff only
  bool m_isSecondTag = false;

  //Controller
  Controller m_controller;
  
  // elevator and intake
  ElevatorIntake m_elevatorIntake;
  LidarReader m_lidar;
  Rollers m_rollers;

  // auto
  AutoLineup m_autoLineup;
  AutoPath m_autoPath;
  AutoDock m_autoDock;
  TwoPieceDock m_twoPieceDock;
  DumbDock m_dumbDock{m_elevatorIntake, m_rollers};
  SadAuto m_sadAuto;
  ThreePiece m_threePiece;
  // TEMP, for testing
  // double m_curVolts;
  // double m_prevTimeTest;
  // wpi::log::DoubleLogEntry m_speedLog;
  // wpi::log::DoubleLogEntry m_voltsLog;
  // END TEMP

  // jetson
  SocketClient m_client;
  bool m_red;
  int m_posVal; // for auto lineup socring positions
  int m_heightVal;
};
