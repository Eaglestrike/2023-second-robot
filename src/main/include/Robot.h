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

#include "Drive/SwerveControl.h"
#include "Drive/SwerveModule.h"
#include "Climb/Climb.h"
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
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected; 

  // inputs
  //frc::Joystick m_lJoy;
  //frc::Joystick m_rJoy;
  frc::Joystick m_xbox;
  // frc::Joystick m_buttonBoard;

  // IMU
  //std::shared_ptr<AHRS> m_navx;

  // swerve
  //SwerveModule m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl;
  //vec::Vector2D m_rFr, m_rBr, m_rFl, m_rBl;
  //std::shared_ptr<SwerveControl> m_swerveController;

  //climb
  Climb m_climb;

  // temp odometry
  //vec::Vector2D m_pos;
};
