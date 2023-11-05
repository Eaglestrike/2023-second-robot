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
      m_swerveFr{SwerveConstants::FR_CONFIG, true, false},
      m_swerveBr{SwerveConstants::BR_CONFIG, true, false},
      m_swerveFl{SwerveConstants::FL_CONFIG, true, false},
      m_swerveBl{SwerveConstants::BL_CONFIG, true, false},
      m_startPos{400, 400},
      m_startAng{0},
      m_joystickAng{0},
      m_odometry{&m_startPos, &m_startAng},
      m_prevTimeTest{0},
      m_client{"10.1.14.107", 5807, 500, 5000},
      m_red{false},
      m_posVal{0},
      m_lidar{true, false},
      m_heightVal{0}
{
    m_lidar.setAutoRequest(true);

  // std::cout << "constructor starting" << std::endl;
  // swerve
  SwerveControl::RefArray<SwerveModule> moduleArray{{m_swerveFr, m_swerveBr, m_swerveFl, m_swerveBl}};
  m_swerveController = new SwerveControl(moduleArray, true, false);

  // navx
  try
  {
    m_navx = std::make_shared<AHRS>(frc::SerialPort::kUSB2);
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

    m_autoLineup.UpdateOdom(pos, ang);
    // m_autoPath.UpdateOdom(pos, ang);

    // UNCOMMENT BELOW
    // frc::SmartDashboard::PutString("Robot pos", pos.toString());
    // frc::SmartDashboard::PutNumber("Robot ang", ang);
    // frc::SmartDashboard::PutBoolean("Cam stale", m_client.IsStale());
    // frc::SmartDashboard::PutBoolean("Cam connection", m_client.HasConn());
    // frc::SmartDashboard::PutData("Field", &m_field);
    // END UNCOMMENT

    // process camera data
    std::vector<double> camData = m_client.GetData();
    if (m_client.HasConn() && !m_client.IsStale()) {
      int camId = static_cast<int>(camData[0]);
      int tagId = static_cast<int>(camData[1]);
      double x = camData[2];
      double y = camData[3];
      double angZ = camData[4];
      long long age = static_cast<long long>(camData[5]);
      unsigned long long uniqueId = static_cast<unsigned long long>(camData[6]);

      // frc::SmartDashboard::PutNumber("camX", x);
      // frc::SmartDashboard::PutNumber("camY", y);

      bool res = camId == 0 && m_odometry.SetCamData({x, y}, angZ, tagId, age, uniqueId);
      // frc::SmartDashboard::PutBoolean("Good ID", res);
      if (!res) {
        tagId = 0;
      } else {
        // std::cout << "good " << tagId << " " << Utils::GetCurTimeMs() << std::endl;
      }
      // frc::SmartDashboard::PutNumber("tag Id", tagId);
    }

    // other odometry
    double angNavX = Utils::DegToRad(m_navx->GetYaw());
    vec::Vector2D velWorld = m_swerveController->GetRobotVelocity(angNavX + m_startAng);

    m_odometry.Periodic(angNavX, velWorld);
    // END ODOMETRY
  }, 5_ms, 2_ms);
  // std::vector<AutoPathInit> v (1, {1.1, *(new EIAutoPath{ElevatorIntake::STOWED, false})});
  // m_testStage = *(new AutoStage(v, 0));
  //std::cout << "constructor done" << std::endl;
}

void Robot::RobotInit()
{
  m_EI.Init();
  m_lidar.Init();
  // initialization
  // frc::SmartDashboard::PutNumber("ang correct kP", SwerveConstants::ANG_CORRECT_P);
  // frc::SmartDashboard::PutNumber("ang correct kI", SwerveConstants::ANG_CORRECT_I);
  // frc::SmartDashboard::PutNumber("ang correct kD", SwerveConstants::ANG_CORRECT_D);

  // kalman filter constants
  // frc::SmartDashboard::PutNumber("KF E0", OdometryConstants::E0);
  // frc::SmartDashboard::PutNumber("KF Q", OdometryConstants::Q);
  // frc::SmartDashboard::PutNumber("KF kAng", OdometryConstants::CAM_TRUST_KANG);
  // frc::SmartDashboard::PutNumber("KF kPos", OdometryConstants::CAM_TRUST_KPOS);
  // frc::SmartDashboard::PutNumber("KF kPosInt", OdometryConstants::CAM_TRUST_KPOSINT);
  // frc::SmartDashboard::PutNumber("Filter Alpha", OdometryConstants::ALPHA);
  // frc::SmartDashboard::PutNumber("Filter maxtime", OdometryConstants::MAX_TIME);

  // frc::SmartDashboard::PutNumber("Delta X", 0);
  // frc::SmartDashboard::PutNumber("Delta Y", 0);
  // frc::SmartDashboard::PutNumber("Delta Ang", 0);
  
  // // starting position

  // m_startPosChooser.SetDefaultOption("Debug", "Debug");
  // m_startPosChooser.AddOption("Blue L", "Blue L");
  // m_startPosChooser.AddOption("Blue M", "Blue M");
  // m_startPosChooser.AddOption("Blue R", "Blue R");
  // m_startPosChooser.AddOption("Red L", "Red L");
  // m_startPosChooser.AddOption("Red M", "Red M");
  // m_startPosChooser.AddOption("Red R", "Red R");
  // frc::SmartDashboard::PutData("Starting pos", &m_startPosChooser);

  // frc::SmartDashboard::PutNumber("trans kP", AutoConstants::TRANS_KP);
  // frc::SmartDashboard::PutNumber("trans kI", AutoConstants::TRANS_KI);
  // frc::SmartDashboard::PutNumber("trans kD", AutoConstants::TRANS_KI);

  // frc::SmartDashboard::PutNumber("ang kP", AutoConstants::ANG_KP);
  // frc::SmartDashboard::PutNumber("ang kI", AutoConstants::ANG_KI);
  // frc::SmartDashboard::PutNumber("ang kD", AutoConstants::ANG_KI);

  // frc::SmartDashboard::PutNumber("trans maxSp", AutoConstants::TRANS_MAXSP);
  // frc::SmartDashboard::PutNumber("trans maxAcc", AutoConstants::TRANS_MAXACC);

  // frc::SmartDashboard::PutNumber("ang maxSp", AutoConstants::ANG_MAXSP);
  // frc::SmartDashboard::PutNumber("ang maxAcc", AutoConstants::ANG_MAXACC);

  m_navx->ZeroYaw();
  m_swerveController->ResetAngleCorrection();

  // // Starts recording to data log
  frc::DataLogManager::Start();

  // // Set up custom log entries
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  m_speedLog = wpi::log::DoubleLogEntry(log, "/ff/swerve/vel");
  m_voltsLog = wpi::log::DoubleLogEntry(log, "/ff/swerve/volts");

  m_client.Init();

  std::cout << "init done" << std::endl;
}

void Robot::initAutoStagesDBG(){
  frc::SmartDashboard::PutBoolean("start", false);
  frc::SmartDashboard::PutBoolean("add EI", false);
  frc::SmartDashboard::PutBoolean("rmv path", false);
  frc::SmartDashboard::PutBoolean("add swrve", false);
  frc::SmartDashboard::PutNumber("cue", 0.0);
  frc::SmartDashboard::GetNumber("strt idx", 0);

  frc::SmartDashboard::PutBoolean("cone", false);
  m_EIautochooser.SetDefaultOption("STOWED", "STOWED");
  m_EIautochooser.AddOption("LOW", "LOW");
  m_EIautochooser.AddOption("MID", "MID");
  m_EIautochooser.AddOption("HIGH", "HIGH");
  m_EIautochooser.AddOption("HP", "HP");
  m_EIautochooser.AddOption("GROUND", "GROUND");
  frc::SmartDashboard::PutData("ei auto chooser", &m_EIautochooser); 
  m_testStage.EnableDBG();
}

void Robot::initEIAutoDBG(){
  frc::SmartDashboard::PutBoolean("cone", false);
  frc::SmartDashboard::PutBoolean("start", false);
  m_EIautochooser.SetDefaultOption("STOWED", "STOWED");
  m_EIautochooser.AddOption("LOW", "LOW");
  m_EIautochooser.AddOption("MID", "MID");
  m_EIautochooser.AddOption("HIGH", "HIGH");
  m_EIautochooser.AddOption("HP", "HP");
  m_EIautochooser.AddOption("GROUND", "GROUND");
  frc::SmartDashboard::PutData("ei auto chooser", &m_EIautochooser);
  m_eiAutoPath.EnableDBG();
}

void Robot::periodicEIAutoDBG(){
  if (frc::SmartDashboard::GetBoolean("start", false)){
    frc::SmartDashboard::PutBoolean("start", false);
    std::string s = m_EIautochooser.GetSelected();
    ElevatorIntake::TargetState targ = ElevatorIntake::TargetState::STOWED;
    if(s=="LOW"){
      targ =ElevatorIntake::TargetState::LOW;
    } else if(s=="MID"){
      targ =ElevatorIntake::TargetState::MID;
    }else if(s=="HIGH"){
      targ =ElevatorIntake::TargetState::HIGH;
    }else if(s=="HP"){
      targ =ElevatorIntake::TargetState::HP;
    }else if(s=="GROUND"){
      targ =ElevatorIntake::TargetState::GROUND;
    }
    m_eiAutoPath = *(new EIAutoPath(targ, frc::SmartDashboard::GetBoolean("cone", false)));
    m_eiAutoPath.EnableDBG();
    m_eiAutoPath.Init(m_EI, m_rollers);
    m_eiAutoPath.Start();
  }
  m_eiAutoPath.AutonomousPeriodic();
}

void Robot::periodicAutoStagesDBG(){
  // add paths to stage
  if (frc::SmartDashboard::GetBoolean("add EI", false)){
    frc::SmartDashboard::PutBoolean("add EI", false);
    std::string s = m_EIautochooser.GetSelected();
    ElevatorIntake::TargetState targ = ElevatorIntake::TargetState::STOWED;
    if(s=="LOW"){
      targ =ElevatorIntake::TargetState::LOW;
    } else if(s=="MID"){
      targ =ElevatorIntake::TargetState::MID;
    }else if(s=="HIGH"){
      targ =ElevatorIntake::TargetState::HIGH;
    }else if(s=="HP"){
      targ =ElevatorIntake::TargetState::HP;
    }else if(s=="GROUND"){
      targ =ElevatorIntake::TargetState::GROUND;
    }
    m_eiAutoPath = *(new EIAutoPath(targ, frc::SmartDashboard::GetBoolean("cone", false)));
    m_eiAutoPath.Init(m_EI, m_rollers);
    m_initpaths.push_back({frc::SmartDashboard::GetNumber("cue", -1.0), m_eiAutoPath});
  }
  // print paths to be in stage
  std::vector<std::string> s;
  for (auto i : m_initpaths){
    s.push_back(i.path.toString());
  }
  std::span<std::string> spn(s);
  frc::SmartDashboard::PutStringArray("paths", spn);

  //remove a path
  if (frc::SmartDashboard::GetBoolean("rmv path", false)){
    frc::SmartDashboard::PutBoolean("rmv path", false);
    m_initpaths.pop_back();
  }

  //initalize the stage with the path list on smart dash
  if (frc::SmartDashboard::GetBoolean("start", false)){
    frc::SmartDashboard::PutBoolean("start", false);
    m_testStage = *(new AutoStage(m_initpaths, frc::SmartDashboard::GetNumber("strt idx", 0)));
    m_testStage.Start();
  }
  m_testStage.AutonomousPeriodic();
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
  m_EI.Periodic();
  m_rollers.Periodic();
  m_lidar.Periodic();
  m_rollers.UpdateLidarData(m_lidar.getData());
  // m_EI.UpdateLidarData(m_lidar.getData());

  // m_eiAutoPath.Periodic();
  // frc::SmartDashboard::PutBoolean("Pos Currently executing", m_autoLineup.GetPosExecuteState() == AutoLineup::EXECUTING_TARGET);
  // frc::SmartDashboard::PutBoolean("Ang Currently executing", m_autoLineup.GetAngExecuteState() == AutoLineup::EXECUTING_TARGET);

  // frc::SmartDashboard::PutBoolean("Pos at target", m_autoLineup.GetPosExecuteState() == AutoLineup::AT_TARGET);
  // frc::SmartDashboard::PutBoolean("Ang at target", m_autoLineup.GetAngExecuteState() == AutoLineup::AT_TARGET);

  // if (m_controller.getPressedOnce(ZERO_DRIVE_PID))
  // {
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
    // m_swerveFl.UpdateShuffleboard();
    // m_swerveFr.UpdateShuffleboard();
    // m_swerveBl.UpdateShuffleboard();
    // m_swerveBr.UpdateShuffleboard();
    // m_swerveController->SetAngleCorrectionPID(kP2, kI2, kD2);

    // double E0 = frc::SmartDashboard::GetNumber("KF E0", OdometryConstants::E0);
    // double Q = frc::SmartDashboard::GetNumber("KF Q", OdometryConstants::Q);
    // double kAng = frc::SmartDashboard::GetNumber("KF kAng", OdometryConstants::CAM_TRUST_KANG);
    // double kPos = frc::SmartDashboard::GetNumber("KF kPos", OdometryConstants::CAM_TRUST_KPOS);
    // double kPosInt = frc::SmartDashboard::GetNumber("KF kPosInt", OdometryConstants::CAM_TRUST_KPOSINT);
    // double alpha = frc::SmartDashboard::GetNumber("Filter Alpha", OdometryConstants::ALPHA);
    // double maxTime = frc::SmartDashboard::GetNumber("Filter maxtime", OdometryConstants::MAX_TIME);

    // m_odometry.SetAlpha(alpha); 
    // m_odometry.SetMaxTime(maxTime);

    // double deltaX = frc::SmartDashboard::GetNumber("Delta X", 0);
    // double deltaY = frc::SmartDashboard::GetNumber("Delta Y", 0);
    // double deltaAng = frc::SmartDashboard::GetNumber("Delta Ang", 0);
    // m_autoLineup.SetPosTarget({deltaX, deltaY}, true);
    // m_autoLineup.SetAngTarget(deltaAng, true);

    // double tkP = frc::SmartDashboard::GetNumber("trans kP", AutoConstants::TRANS_KP);
    // double tkI = frc::SmartDashboard::GetNumber("trans kI", AutoConstants::TRANS_KI);
    // double tkD = frc::SmartDashboard::GetNumber("trans kD", AutoConstants::TRANS_KI);

    // double akP = frc::SmartDashboard::GetNumber("ang kP", AutoConstants::ANG_KP);
    // double akI = frc::SmartDashboard::GetNumber("ang kI", AutoConstants::ANG_KI);
    // double akD = frc::SmartDashboard::GetNumber("ang kD", AutoConstants::ANG_KI);

    // m_autoLineup.SetPosPID(tkP, tkI, tkD);
    // m_autoLineup.SetAngPID(akP, akI, akD);
    // m_autoPath.SetPosPID(tkP, tkI, tkD);
    // m_autoPath.SetAngPID(akP, akI, akD);

    // double tMaxSp = frc::SmartDashboard::GetNumber("trans maxSp", AutoConstants::TRANS_MAXSP);
    // double tMaxAcc = frc::SmartDashboard::GetNumber("trans maxAcc", AutoConstants::TRANS_MAXACC);

    // double aMaxSp = frc::SmartDashboard::GetNumber("ang maxSp", AutoConstants::ANG_MAXSP);
    // double aMaxAcc = frc::SmartDashboard::GetNumber("ang maxAcc", AutoConstants::ANG_MAXACC);

    // m_autoLineup.SetPosFF({tMaxSp, tMaxAcc});
    // m_autoLineup.SetAngFF({aMaxSp, aMaxAcc});

    // m_odometry.SetKFTerms(E0, Q, kAng, kPos, kPosInt, maxTime);
  // }

  // if (m_controller.getPressedOnce(ZERO_YAW))
  // {
  //   m_navx->ZeroYaw();
  //   m_swerveController->ResetAngleCorrection(m_startAng);
  //   m_odometry.Reset();
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
void Robot::AutonomousInit() {
  initEIAutoDBG();
  // initAutoStagesDBG();
  //m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);
  // TESTING CODE
  // MAKE SURE BLUE RIGHT OR ELSE ROBOT WILL UNALIVE ITSELF
  // m_autoPath.AddPoses(AutoPaths::BIG_BOY);
  // m_autoPath.StartMove();

  // TODO: where did the elevator intake member go?
  
 // m_auto_manager{&m_swerveController};
}

void Robot::AutonomousPeriodic(){
  periodicEIAutoDBG();
  // periodicAutoStagesDBG();

  // m_swerve_auto.AutonomousPeriodic();
  // m_auto_manager.AutonomousPeriodic();
  // m_auto_manager{&m_swerveController};
  AutoPaths::SwervePose swp_1 = {0.0, 10, 20, 30, 30, 40, 10};
  AutoPaths::SwervePose swp_2 = {0.0, 10, 20, 30, 30, 40, 10};
  AutoPaths::SwervePose swp_3 = {0.0, 10, 20, 30, 30, 40, 10};

  

}

void Robot::TeleopInit() {
  m_swerveController->SetFeedForward(0, 1, 0);
  // m_swerveController->SetAngleCorrectionPID(SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D);
  // m_autoLineup.SetPosFF({.maxSpeed = AutoConstants::TRANS_MAXSP, .maxAccel = AutoConstants::TRANS_MAXACC});
  // m_autoLineup.SetAngFF({.maxSpeed = AutoConstants::ANG_MAXSP, .maxAccel = AutoConstants::ANG_MAXACC});
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

  // get auto lineup pos
  int posVal = m_controller.getValue(ControllerMapData::SCORING_POS, 0);
  if (posVal) {
    m_posVal = posVal;
  }
  int heightVal = m_controller.getValue(ControllerMapData::GET_LEVEL, 0);
  if (heightVal) {
    m_heightVal = heightVal;
  }

  if (m_posVal && m_heightVal) {
    FieldConstants::ScorePair scorePair = Utils::GetScoringPos(m_posVal, m_heightVal, m_red);
    double ang = m_joystickAng;

    vec::Vector2D scorePos = scorePair.first;
    double lidarOffset = scorePair.second;
    
    // if hascone
    //    scorePos -= {0, blue ? lidarReading - lidarOffset : lidarOffset - lidarReading}
    // else if hascube
    //    scorePos += {0, blue ? lidarReading - lidarOffset : lidarOffset - lidarReading}

    // commented out right now because untuned, if this executed the robot will fly to (0, 0)
    // m_autoLineup.SetAbsTargetPose(scorePos, ang);
  }

  // trim, offset shold be opposite direction of offset so it moves correct direction
  double trimX = -m_controller.getValue(ControllerMapData::GET_TRIM_X, 0.0);
  double trimY = -m_controller.getValue(ControllerMapData::GET_TRIM_Y, 0.0);
  vec::Vector2D offset = {trimX, trimY};
  if (m_red) {
    offset = rotate(offset, M_PI / 2);
  } else {
    offset = rotate(offset, -M_PI / 2);
  }
  m_startPos += offset;

  // cancel auto lineup if joysticks move
  if (!Utils::NearZero(setVel)) {
    m_autoLineup.StopPos();
  }

  AutoLineup::ExecuteState curPosAutoState = m_autoLineup.GetPosExecuteState();
  AutoLineup::ExecuteState curAngAutoState = m_autoLineup.GetAngExecuteState();

  if (m_controller.getPressedOnce(START_POS_AUTO)) {
    if (curPosAutoState != AutoLineup::EXECUTING_TARGET) {
      m_autoLineup.StartPosMove();
    } else {
      m_autoLineup.StopPos();
    }
  }

  if (m_controller.getPressedOnce(START_ANG_AUTO)) {
    if (curAngAutoState != AutoLineup::EXECUTING_TARGET) {
      m_autoLineup.StartAngMove();
    } else {
      m_autoLineup.StopAng();
    }
  }

  if (curPosAutoState == AutoLineup::EXECUTING_TARGET || curAngAutoState == AutoLineup::EXECUTING_TARGET) {
    vec::Vector2D driveVel = m_autoLineup.GetVel();
    double angVel = m_autoLineup.GetAngVel();

    frc::SmartDashboard::PutString("Auto drive vel", driveVel.toString());
    frc::SmartDashboard::PutNumber("Auto ang vel", angVel);

    m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);
    m_swerveController->SetRobotVelocity(driveVel, angVel, curYaw, deltaT);
  } else {
    // std::cout << "set vel:  " << setVel.toString() << std::endl;
    // m_swerveController->SetAngleCorrectionPID(SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D);
    m_swerveController->SetFeedForward(0, 1, 0);
    m_swerveController->SetRobotVelocityTele(setVel, w, curYaw, deltaT, m_joystickAng);
  }

  m_autoLineup.Periodic();
  m_swerveController->Periodic();

  // frc::SmartDashboard::PutString("pos:", m_pos.toString());
  // frc::SmartDashboard::PutString("vel:", vel.toString());
  // frc::SmartDashboard::PutString("setVel:", setVel.toString());
  // frc::SmartDashboard::PutNumber("setAngVel:", w);
  m_prevTime = curTime;
  // m_intake.TeleopPeriodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  m_autoLineup.StopPos();
  m_autoLineup.StopAng();
  
  // get position offsets (sorry for bad if statements)
  std::string m_selected = m_startPosChooser.GetSelected();
  if (m_selected == "Debug") {
    m_startAng = FieldConstants::DEBUG_ANG;
    m_startPos = FieldConstants::DEBUG_POS;
    m_joystickAng = FieldConstants::DEBUG_JANG;
    m_red = false;
  } else if (m_selected == "Blue L") {
    m_startAng = FieldConstants::BL_ANG;
    m_startPos = FieldConstants::BL_POS;
    m_joystickAng = FieldConstants::BL_JANG;
    m_red = false;
  } else if (m_selected == "Blue M") {
    m_startAng = FieldConstants::BM_ANG;
    m_startPos = FieldConstants::BM_POS;
    m_joystickAng = FieldConstants::BM_JANG;
    m_red = false;
  } else if (m_selected == "Blue R") {
    m_startAng = FieldConstants::BR_ANG;
    m_startPos = FieldConstants::BR_POS;
    m_joystickAng = FieldConstants::BR_JANG;
    m_red = false;
  } else if (m_selected == "Red L") {
    m_startAng = FieldConstants::RL_ANG;
    m_startPos = FieldConstants::RL_POS;
    m_joystickAng = FieldConstants::RL_JANG;
    m_red = true;
  } else if (m_selected == "Red M") {
    m_startAng = FieldConstants::RM_ANG;
    m_startPos = FieldConstants::RM_POS;
    m_joystickAng = FieldConstants::RM_JANG;
    m_red = true;
  } else if (m_selected == "Red R") {
    m_startAng = FieldConstants::RR_ANG;
    m_startPos = FieldConstants::RR_POS;
    m_joystickAng = FieldConstants::RR_JANG;
    m_red = true;
  } else {
    m_startAng = FieldConstants::DEBUG_ANG;
    m_startPos = FieldConstants::DEBUG_POS;
    m_joystickAng = FieldConstants::DEBUG_JANG;
  }
}

void Robot::TestInit() {
  m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);

  m_curVolts = 0;
}

void Robot::TestPeriodic() {
  // if (m_controller.getPressedOnce(START_POS_AUTO)) {
  //   if (m_autoLineup.GetAngExecuteState() == AutoLineup::NOT_EXECUTING) {
  //     m_autoLineup.StartAngMove();
  //   } else {
  //     m_autoLineup.StopAng();
  //   }
  // }

  // vec::Vector2D vel = m_autoLineup.GetVel();
  // double angVel = m_autoLineup.GetAngVel();
  double curYaw = m_odometry.GetAng();

  // frc::SmartDashboard::PutString("AutoVel", vel.toString());
  // frc::SmartDashboard::PutNumber("angVel", angVel);

  if (m_curVolts >= 5) {
    m_curVolts = 5;
  }

  m_swerveController->SetRobotVelocity({m_curVolts, 0}, 0, curYaw, 0.02);

  double curS = Utils::GetCurTimeS();
  if (curS - m_prevTimeTest > 0.2) {
    m_prevTimeTest = curS;
    m_curVolts += 0.5;
  }

  // m_autoLineup.Periodic();
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
