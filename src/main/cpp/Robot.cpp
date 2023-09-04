// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  lidar_.RequestData();
  lidar_.Periodic();

  LidarReader::LidarData lidarData = lidar_.getData();
  
  frc::SmartDashboard::PutNumber("cube pos", lidarData.cubePos);
  frc::SmartDashboard::PutNumber("cone pos", lidarData.conePos);
  frc::SmartDashboard::PutBoolean("has cone", lidarData.hasCone);
  frc::SmartDashboard::PutBoolean("has cube", lidarData.hasCube);
  frc::SmartDashboard::PutBoolean("valid data", lidarData.isValid);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
