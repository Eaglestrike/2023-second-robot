// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Drive/DriveConstants.h"
#include "Controller/ControllerMap.h"
#include "GeneralConstants.h"
#include "Util/Mathutil.h"

using namespace Actions;

Robot::Robot():
      m_prevTime{0},
      m_swerveFr{SwerveConstants::FR_DRIVE_ID, SwerveConstants::FR_TURN_ID, SwerveConstants::FR_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::FR_DRIVE_INVERTED, SwerveConstants::FR_ENCODER_INVERTED, SwerveConstants::FR_ANG_INVERTED, SwerveConstants::FR_OFFSET},
      m_swerveBr{SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_TURN_ID, SwerveConstants::BR_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::BR_DRIVE_INVERTED, SwerveConstants::BR_ENCODER_INVERTED, SwerveConstants::BR_ANG_INVERTED, SwerveConstants::BR_OFFSET},
      m_swerveFl{SwerveConstants::FL_DRIVE_ID, SwerveConstants::FL_TURN_ID, SwerveConstants::FL_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::FL_DRIVE_INVERTED, SwerveConstants::FL_ENCODER_INVERTED, SwerveConstants::FL_ANG_INVERTED, SwerveConstants::FL_OFFSET},
      m_swerveBl{SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_TURN_ID, SwerveConstants::BL_ENCODER_ID, SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D, SwerveConstants::BL_DRIVE_INVERTED, SwerveConstants::BL_ENCODER_INVERTED, SwerveConstants::BL_ANG_INVERTED, SwerveConstants::BL_OFFSET},
      m_rFr{SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE},
      m_rBr{-SwerveConstants::CENTER_TO_EDGE, -SwerveConstants::CENTER_TO_EDGE},
      m_rFl{SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE},
      m_rBl{-SwerveConstants::CENTER_TO_EDGE, SwerveConstants::CENTER_TO_EDGE},
      m_startPos{400, 400},
      m_startAng{0},
      m_joystickAng{0},
      m_odometry{&m_startPos, &m_startAng},
      m_autoDrive{&m_odometry},
      m_client{"10.1.14.107", 5807, 500, 5000}
{
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  std::array<vec::Vector2D, 4> radiiArray{{m_rFr, m_rBr, m_rFl, m_rBl}};
  m_swerveController = new SwerveControl(moduleArray, radiiArray, 0, 1, 0);

  // navx
  try
  {
    m_navx = std::make_shared<AHRS>(frc::SerialPort::kMXP);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  AddPeriodic([&](){
    // ODOMETRY
    vec::Vector2D pos = m_odometry.GetPosition();
    double ang = m_odometry.GetAng();

    m_field.SetRobotPose(units::meter_t{pos.x()}, units::meter_t{pos.y()}, units::radian_t{ang});

    // UNCOMMENT BELOW
    // frc::SmartDashboard::PutString("Filter pos", pos.toString());
    // frc::SmartDashboard::PutNumber("Filter ang", ang);
    // frc::SmartDashboard::PutBoolean("Cam stale", m_client.IsStale());
    // frc::SmartDashboard::PutBoolean("Cam connection", m_client.HasConn());
    // frc::SmartDashboard::PutData("Field", &m_field);
    // END UNCOMMENT

    // process camera data
    std::vector<double> camData = m_client.GetData();
    if (m_client.HasConn() && !m_client.IsStale()) {
      int tagId = static_cast<int>(camData[1]);
      double x = camData[2];
      double y = camData[3];
      double angZ = camData[4];
      long long age = static_cast<long long>(camData[5]);
      unsigned long long uniqueId = static_cast<unsigned long long>(camData[6]);

      // frc::SmartDashboard::PutNumber("camX", x);
      // frc::SmartDashboard::PutNumber("camY", y);

      bool res = m_odometry.SetCamData({x, y}, angZ, tagId, age, uniqueId);
      // frc::SmartDashboard::PutBoolean("Good ID", res);
      if (!res) {
        tagId = 0;
      } else {
        // std::cout << "good " << tagId << " " << Utils::GetCurTimeMs() << std::endl;
      }
      frc::SmartDashboard::PutNumber("tag Id", tagId);
    }

    // other odometry
    double angNavX = Utils::DegToRad(m_navx->GetYaw());
    vec::Vector2D velWorld = m_swerveController->GetRobotVelocity(angNavX + m_startAng);

    m_odometry.Periodic(angNavX, velWorld);
    // END ODOMETRY
  }, 5_ms, 2_ms);
}

void Robot::RobotInit()
{
  // frc::SmartDashboard::PutNumber("wheel kP", SwerveConstants::TURN_P);
  // frc::SmartDashboard::PutNumber("wheel kI", SwerveConstants::TURN_I);
  // frc::SmartDashboard::PutNumber("wheel kD", SwerveConstants::TURN_D);
  // frc::SmartDashboard::PutNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  // frc::SmartDashboard::PutNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  // frc::SmartDashboard::PutNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);


  // set PID for wheels and angle correction
  m_swerveFl.SetPID(SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D);
  m_swerveFr.SetPID(SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D);
  m_swerveBl.SetPID(SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D);
  m_swerveBr.SetPID(SwerveConstants::TURN_P, SwerveConstants::TURN_I, SwerveConstants::TURN_D);
  m_swerveController->SetAngleCorrectionPID(SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D);

  // kalman filter constants
  // frc::SmartDashboard::PutNumber("KF E0", OdometryConstants::E0);
  // frc::SmartDashboard::PutNumber("KF Q", OdometryConstants::Q);
  // frc::SmartDashboard::PutNumber("KF kAng", OdometryConstants::CAM_TRUST_KANG);
  // frc::SmartDashboard::PutNumber("KF kPos", OdometryConstants::CAM_TRUST_KPOS);
  // frc::SmartDashboard::PutNumber("KF kPosInt", OdometryConstants::CAM_TRUST_KPOSINT);
  frc::SmartDashboard::PutNumber("Filter Alpha", OdometryConstants::ALPHA);
  frc::SmartDashboard::PutNumber("Filter maxtime", OdometryConstants::MAX_TIME);

  // frc::SmartDashboard::PutNumber("Delta X", 0);
  // frc::SmartDashboard::PutNumber("Delta Y", 0);
  frc::SmartDashboard::PutNumber("Delta Ang", 0);

  // starting position
  m_startPosChooser.SetDefaultOption("Debug", "Debug");
  m_startPosChooser.AddOption("Blue L", "Blue L");
  m_startPosChooser.AddOption("Blue M", "Blue M");
  m_startPosChooser.AddOption("Blue R", "Blue R");
  m_startPosChooser.AddOption("Red L", "Red L");
  m_startPosChooser.AddOption("Red M", "Red M");
  m_startPosChooser.AddOption("Red R", "Red R");
  frc::SmartDashboard::PutData("Starting pos", &m_startPosChooser);

  // initialization
  m_navx->ZeroYaw();
  m_swerveController->ResetAngleCorrection();

  // Starts recording to data log
  frc::DataLogManager::Start();

  // Set up custom log entries
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  m_speedLog = wpi::log::DoubleLogEntry(log, "/ff/swerve/vel");
  m_voltsLog = wpi::log::DoubleLogEntry(log, "/ff/swerve/volts");

  m_client.Init();
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
  frc::SmartDashboard::PutBoolean("Ang Currently executing", m_autoDrive.GetAngExecuteState() != AutoDrive::NOT_EXECUTING);

  if (m_controller.getPressed(ZERO_DRIVE_PID))
  {
    // double kP = frc::SmartDashboard::GetNumber("wheel kP", SwerveConstants::TURN_P);
    // double kI = frc::SmartDashboard::GetNumber("wheel kI", SwerveConstants::TURN_I);
    // double kD = frc::SmartDashboard::GetNumber("wheel kD", SwerveConstants::TURN_D);

    // m_swerveFl.SetPID(kP, kI, kD);
    // m_swerveFr.SetPID(kP, kI, kD);
    // m_swerveBl.SetPID(kP, kI, kD);
    // m_swerveBr.SetPID(kP, kI, kD);

    // double kP2 = frc::SmartDashboard::GetNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
    // double kI2 = frc::SmartDashboard::GetNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
    // double kD2 = frc::SmartDashboard::GetNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);
    // m_swerveController->SetAngleCorrectionPID(kP2, kI2, kD2);

    // double E0 = frc::SmartDashboard::GetNumber("KF E0", OdometryConstants::E0);
    // double Q = frc::SmartDashboard::GetNumber("KF Q", OdometryConstants::Q);
    // double kAng = frc::SmartDashboard::GetNumber("KF kAng", OdometryConstants::CAM_TRUST_KANG);
    // double kPos = frc::SmartDashboard::GetNumber("KF kPos", OdometryConstants::CAM_TRUST_KPOS);
    // double kPosInt = frc::SmartDashboard::GetNumber("KF kPosInt", OdometryConstants::CAM_TRUST_KPOSINT);
    double alpha = frc::SmartDashboard::GetNumber("Filter Alpha", OdometryConstants::ALPHA);
    double maxTime = frc::SmartDashboard::GetNumber("Filter maxtime", OdometryConstants::MAX_TIME);

    m_odometry.SetAlpha(alpha); 
    m_odometry.SetMaxTime(maxTime);

    double deltaX = frc::SmartDashboard::GetNumber("Delta X", 0);
    double deltaY = frc::SmartDashboard::GetNumber("Delta Y", 0);
    // double deltaAng = frc::SmartDashboard::GetNumber("Delta Ang", 0);
    // m_autoDrive.SetRelTargetPose({0, 0}, deltaAng);
    m_autoDrive.SetRelTargetPose({deltaX, deltaY}, 0);

    // m_odometry.SetKFTerms(E0, Q, kAng, kPos, kPosInt, maxTime);
  }

  if (m_controller.getPressed(ZERO_YAW))
  {
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection(m_startAng);
    m_odometry.Reset();
  }

  // delete later
  // if (m_controller.getPressed(ZERO_AUTO)) {
  // }

  // frc::SmartDashboard::PutNumber("fl raw encoder", m_swerveFl.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("fr raw encoder", m_swerveFr.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("bl raw encoder", m_swerveBl.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("br raw encoder", m_swerveBr.GetRawEncoderReading());

  // frc::SmartDashboard::PutString("fl velocity", m_swerveFl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("fr velocity", m_swerveFr.GetVelocity().toString());
  // frc::SmartDashboard::PutString("bl velocity", m_swerveBl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("br velocity", m_swerveBr.GetVelocity().toString());
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
void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit() {
  m_swerveController->SetFeedForward(0, 1, 0);
}

void Robot::TeleopPeriodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.1);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.1);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.1);

  double vx = std::clamp(lx, -1.0, 1.0) * 12.0;
  double vy = std::clamp(ly, -1.0, 1.0) * 12.0;
  double w = -std::clamp(rx, -1.0, 1.0) * 12.0;

  double curYaw = m_odometry.GetAng();

  // frc::SmartDashboard::PutNumber("curYaw", curYaw);

  vec::Vector2D setVel = {-vy, -vx};

  // cancel auto if joysticks move
  if (!Utils::NearZero(setVel)) {
    m_autoDrive.StopPos();
  }

  if (m_controller.getPressed(START_AUTO)) {
    if (m_autoDrive.GetPosExecuteState() == AutoDrive::NOT_EXECUTING) {
      m_autoDrive.StartPosMove();
    } else {
      m_autoDrive.StopPos();
    }
  }

  if (m_autoDrive.GetPosExecuteState() == AutoDrive::NOT_EXECUTING) {
    m_swerveController->SetRobotVelocityAbs(setVel, w, curYaw, deltaT, m_joystickAng);
  } else {
    vec::Vector2D driveVel = m_autoDrive.GetVel();
    double angVel = m_autoDrive.GetAngVel();

    m_swerveController->SetRobotVelocity(driveVel, angVel, curYaw, deltaT);
  }

  m_autoDrive.Periodic();
  m_swerveController->Periodic();

  // frc::SmartDashboard::PutString("pos:", m_pos.toString());
  // frc::SmartDashboard::PutString("vel:", vel.toString());
  // frc::SmartDashboard::PutString("setVel:", setVel.toString());
  // frc::SmartDashboard::PutNumber("setAngVel:", w);
  m_prevTime = curTime;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  // get position offsets (sorry for bad if statements)
  std::string m_selected = m_startPosChooser.GetSelected();
  if (m_selected == "Debug") {
    m_startAng = FieldConstants::DEBUG_ANG;
    m_startPos = FieldConstants::DEBUG_POS;
    m_joystickAng = FieldConstants::DEBUG_JANG;
  } else if (m_selected == "Blue L") {
    m_startAng = FieldConstants::BL_ANG;
    m_startPos = FieldConstants::BL_POS;
    m_joystickAng = FieldConstants::BL_JANG;
  } else if (m_selected == "Blue M") {
    m_startAng = FieldConstants::BM_ANG;
    m_startPos = FieldConstants::BM_POS;
    m_joystickAng = FieldConstants::BM_JANG;
  } else if (m_selected == "Blue R") {
    m_startAng = FieldConstants::BR_ANG;
    m_startPos = FieldConstants::BR_POS;
    m_joystickAng = FieldConstants::BR_JANG;
  } else if (m_selected == "Red L") {
    m_startAng = FieldConstants::RL_ANG;
    m_startPos = FieldConstants::RL_POS;
    m_joystickAng = FieldConstants::RL_JANG;
  } else if (m_selected == "Red M") {
    m_startAng = FieldConstants::RM_ANG;
    m_startPos = FieldConstants::RM_POS;
    m_joystickAng = FieldConstants::RM_JANG;
  } else if (m_selected == "Red R") {
    m_startAng = FieldConstants::RR_ANG;
    m_startPos = FieldConstants::RR_POS;
    m_joystickAng = FieldConstants::RR_JANG;
  } else {
    m_startAng = FieldConstants::DEBUG_ANG;
    m_startPos = FieldConstants::DEBUG_POS;
    m_joystickAng = FieldConstants::DEBUG_JANG;
  }
}

void Robot::TestInit() {
  m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);
  m_autoDrive.SetFFPos({1, 1});
  m_autoDrive.SetFFAng({5, 5});

  m_curVolts = 0;
}

void Robot::TestPeriodic() {
  // if (m_controller.getPressed(START_AUTO)) {
  //   if (m_autoDrive.GetAngExecuteState() == AutoDrive::NOT_EXECUTING) {
  //     m_autoDrive.StartAngMove();
  //   } else {
  //     m_autoDrive.StopAng();
  //   }
  // }

  // vec::Vector2D vel = m_autoDrive.GetVel();
  // double angVel = m_autoDrive.GetAngVel();
  double curYaw = m_odometry.GetAng();

  // frc::SmartDashboard::PutString("AutoVel", vel.toString());
  // frc::SmartDashboard::PutNumber("angVel", angVel);

  if (m_curVolts >= 5) {
    m_curVolts = 5;
  }

  m_swerveController->SetRobotVelocity({m_curVolts, 0}, 0, curYaw, 0.02);

  double curS = Utils::GetCurTimeS();
  if (curS - m_prevTime > 0.2) {
    m_prevTime = curS;
    m_curVolts += 0.1;
  }

  // m_autoDrive.Periodic();
  vec::Vector2D robotVel = m_swerveController->GetRobotVelocity(curYaw);

  auto curTime = frc::Timer::GetFPGATimestamp();
  auto timeus = curTime.convert<units::microsecond>(); 

  m_voltsLog.Append(m_curVolts, timeus.value());
  m_speedLog.Append(magn(robotVel), timeus.value());
  m_swerveController->Periodic();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
