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
      m_startPos{0, 0},
      m_startAng{0},
      m_joystickAng{0},
      m_odometry{&m_startPos, &m_startAng},
      m_client{"10.1.14.107", 5807, 500, 5000},
      m_red{false},
      m_posVal{0},
      m_heightVal{0},
      m_lidar{true, false}
{
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

  // lidar
  m_lidar.setAutoRequest(true);

  AddPeriodic([&](){
    // ODOMETRY
    vec::Vector2D pos = m_odometry.GetPosition();
    double ang = m_odometry.GetAng();
    m_field.SetRobotPose(units::meter_t{pos.x()}, units::meter_t{pos.y()}, units::radian_t{ang});

    m_autoLineup.UpdateOdom(pos, ang);
    m_autoPath.UpdateOdom(pos, ang);

    // UNCOMMENT BELOW
    frc::SmartDashboard::PutString("Robot pos", pos.toString());
    frc::SmartDashboard::PutNumber("Robot ang", ang);
    frc::SmartDashboard::PutBoolean("Cam stale", m_client.IsStale());
    frc::SmartDashboard::PutBoolean("Cam connection", m_client.HasConn());
    frc::SmartDashboard::PutData("Field", &m_field);
    // END UNCOMMENT

    int expectedId = Utils::GetExpectedTagId(m_posVal, m_red);

    // process camera data
    std::vector<double> camData = m_client.GetData();
    bool isCorrecting = false;
    if (!m_isAutoLineup && !m_isTrimming && m_client.HasConn() && !m_client.IsStale()) {
      int camId = static_cast<int>(camData[0]);
      int tagId = static_cast<int>(camData[1]);
      double x = camData[2];
      double y = camData[3];
      double angZ = camData[4];
      long long age = static_cast<long long>(camData[5]);
      unsigned long long uniqueId = static_cast<unsigned long long>(camData[6]);

      frc::SmartDashboard::PutNumber("camX", x);
      frc::SmartDashboard::PutNumber("camY", y);

      bool res = false;
      if (camId != 0 && m_isFirstTag) {
        res = m_odometry.SetCamData({x, y}, angZ, tagId, age, uniqueId);
      } 

      // frc::SmartDashboard::PutBoolean("Good ID", res);
      if (!res) {
        tagId = 0;
      } else {
        isCorrecting = true;
        // std::cout << "good " << tagId << " " << Utils::GetCurTimeMs() << std::endl;
      }
      frc::SmartDashboard::PutNumber("tag Id", tagId);
      m_isFirstTag = true;
    } else {
      m_isFirstTag = false;
    }
    // frc::SmartDashboard::PutBoolean("Is correcting", isCorrecting);

    // other odometry
    double angNavX = Utils::DegToRad(m_navx->GetYaw());
    vec::Vector2D velWorld = m_swerveController->GetRobotVelocity(angNavX + m_startAng);

    m_odometry.Periodic(angNavX, velWorld);
    // END ODOMETRY
  }, 5_ms, 2_ms);
}

void Robot::RobotInit()
{
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
  frc::SmartDashboard::PutNumber("Filter Alpha", OdometryConstants::ALPHA);
  // frc::SmartDashboard::PutNumber("Filter maxtime", OdometryConstants::MAX_TIME);

  // frc::SmartDashboard::PutNumber("Delta X", 0);
  // frc::SmartDashboard::PutNumber("Delta Y", 0);
  // frc::SmartDashboard::PutNumber("Delta Ang", 0);

  // starting position
  m_startPosChooser.SetDefaultOption("Debug", "Debug");
  m_startPosChooser.AddOption("Blue L", "Blue L");
  m_startPosChooser.AddOption("Blue M", "Blue M");
  m_startPosChooser.AddOption("Blue R", "Blue R");
  m_startPosChooser.AddOption("Red L", "Red L");
  m_startPosChooser.AddOption("Red M", "Red M");
  m_startPosChooser.AddOption("Red R", "Red R");
  frc::SmartDashboard::PutData("Starting pos", &m_startPosChooser);

  // frc::SmartDashboard::PutNumber("trans kP", AutoConstants::TRANS_KP);
  // frc::SmartDashboard::PutNumber("trans kI", AutoConstants::TRANS_KI);
  // frc::SmartDashboard::PutNumber("trans kD", AutoConstants::TRANS_KD);

  // frc::SmartDashboard::PutNumber("ang kP", AutoConstants::ANG_KP);
  // frc::SmartDashboard::PutNumber("ang kI", AutoConstants::ANG_KI);
  // frc::SmartDashboard::PutNumber("ang kD", AutoConstants::ANG_KD);

  // frc::SmartDashboard::PutNumber("trans maxSp", AutoConstants::TRANS_MAXSP);
  // frc::SmartDashboard::PutNumber("trans maxAcc", AutoConstants::TRANS_MAXACC);

  // frc::SmartDashboard::PutNumber("ang maxSp", AutoConstants::ANG_MAXSP);
  // frc::SmartDashboard::PutNumber("ang maxAcc", AutoConstants::ANG_MAXACC);

  // frc::SmartDashboard::PutNumber("1 Mid X", FieldConstants::BLUE_SCORING_POS[0][1].first.x());
  // frc::SmartDashboard::PutNumber("1 High X", FieldConstants::BLUE_SCORING_POS[0][2].first.x());
  // frc::SmartDashboard::PutNumber("1 Mid Y", FieldConstants::BLUE_SCORING_POS[0][1].first.y());
  // frc::SmartDashboard::PutNumber("1 High Y", FieldConstants::BLUE_SCORING_POS[0][2].first.y());

  m_navx->ZeroYaw();
  m_swerveController->ResetAngleCorrection();

  // Starts recording to data log
  // frc::DataLogManager::Start();

  // Set up custom log entries
  // wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  // m_speedLog = wpi::log::DoubleLogEntry(log, "/ff/swerve/vel");
  // m_voltsLog = wpi::log::DoubleLogEntry(log, "/ff/swerve/volts");

  m_lidar.Init();
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
  frc::SmartDashboard::PutBoolean("Pos Currently executing", m_autoLineup.GetPosExecuteState() == AutoLineup::EXECUTING_TARGET);
  frc::SmartDashboard::PutBoolean("Ang Currently executing", m_autoLineup.GetAngExecuteState() == AutoLineup::EXECUTING_TARGET);

  frc::SmartDashboard::PutBoolean("Pos at target", m_autoLineup.GetPosExecuteState() == AutoLineup::AT_TARGET);
  frc::SmartDashboard::PutBoolean("Ang at target", m_autoLineup.GetAngExecuteState() == AutoLineup::AT_TARGET);

  if (m_controller.getPressedOnce(ZERO_DRIVE_PID))
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
    double alpha = frc::SmartDashboard::GetNumber("Filter Alpha", OdometryConstants::ALPHA);
    // double maxTime = frc::SmartDashboard::GetNumber("Filter maxtime", OdometryConstants::MAX_TIME);

    m_odometry.SetAlpha(alpha); 
    // m_odometry.SetMaxTime(maxTime);

    // double deltaX = frc::SmartDashboard::GetNumber("Delta X", 0);
    // double deltaY = frc::SmartDashboard::GetNumber("Delta Y", 0);
    // double deltaAng = frc::SmartDashboard::GetNumber("Delta Ang", 0);
    // m_autoLineup.SetPosTarget({deltaX, deltaY}, true);
    // m_autoLineup.SetAngTarget(deltaAng, true);

    double tkP = frc::SmartDashboard::GetNumber("trans kP", AutoConstants::TRANS_KP);
    double tkI = frc::SmartDashboard::GetNumber("trans kI", AutoConstants::TRANS_KI);
    double tkD = frc::SmartDashboard::GetNumber("trans kD", AutoConstants::TRANS_KD);

    double akP = frc::SmartDashboard::GetNumber("ang kP", AutoConstants::ANG_KP);
    double akI = frc::SmartDashboard::GetNumber("ang kI", AutoConstants::ANG_KI);
    double akD = frc::SmartDashboard::GetNumber("ang kD", AutoConstants::ANG_KD);

    m_autoLineup.SetPosPID(tkP, tkI, tkD);
    m_autoLineup.SetAngPID(akP, akI, akD);
    m_autoPath.SetPosPID(tkP, tkI, tkD);
    m_autoPath.SetAngPID(akP, akI, akD);

    double tMaxSp = frc::SmartDashboard::GetNumber("trans maxSp", AutoConstants::TRANS_MAXSP);
    double tMaxAcc = frc::SmartDashboard::GetNumber("trans maxAcc", AutoConstants::TRANS_MAXACC);

    double aMaxSp = frc::SmartDashboard::GetNumber("ang maxSp", AutoConstants::ANG_MAXSP);
    double aMaxAcc = frc::SmartDashboard::GetNumber("ang maxAcc", AutoConstants::ANG_MAXACC);

    m_autoLineup.SetPosFF({tMaxSp, tMaxAcc});
    m_autoLineup.SetAngFF({aMaxSp, aMaxAcc});

    // m_odometry.SetKFTerms(E0, Q, kAng, kPos, kPosInt, maxTime);

    // double mx = frc::SmartDashboard::GetNumber("1 Mid X", FieldConstants::BLUE_SCORING_POS[0][1].first.x());
    // double hx = frc::SmartDashboard::GetNumber("1 High X", FieldConstants::BLUE_SCORING_POS[0][2].first.x());
    // double my = frc::SmartDashboard::GetNumber("1 Mid Y", FieldConstants::BLUE_SCORING_POS[0][1].first.y());
    // double hy = frc::SmartDashboard::GetNumber("1 High Y", FieldConstants::BLUE_SCORING_POS[0][2].first.y());

    m_elevatorIntake.UpdateShuffleboard();
  }

  if (m_controller.getPressedOnce(ZERO_YAW))
  {
    m_navx->ZeroYaw();
    m_swerveController->ResetAngleCorrection(m_startAng);
    m_odometry.Reset();
  }

  // frc::SmartDashboard::PutNumber("fl raw encoder", m_swerveFl.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("fr raw encoder", m_swerveFr.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("bl raw encoder", m_swerveBl.GetRawEncoderReading());
  // frc::SmartDashboard::PutNumber("br raw encoder", m_swerveBr.GetRawEncoderReading());

  // frc::SmartDashboard::PutString("fl velocity", m_swerveFl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("fr velocity", m_swerveFr.GetVelocity().toString());
  // frc::SmartDashboard::PutString("bl velocity", m_swerveBl.GetVelocity().toString());
  // frc::SmartDashboard::PutString("br velocity", m_swerveBr.GetVelocity().toString());
  // if (m_controller.getPressed(ELEVATOR_UPDATE)) {
  //   // elevator_.UpdateShuffleboard();
  // }

  // if (m_controller.getPressed(ELEVATOR_EXTEND_STOWED)) {
  //   // elevator_.Stow();
  // }
  // else if (m_controller.getPressed(ELEVATOR_EXTEND_LOW)) {
  //   // elevator_.ExtendLow();
  // }
  // else if (m_controller.getPressed(ELEVATOR_EXTEND_MID)) {
  //   // elevator_.ExtendMid();
  // }
  // else if (m_controller.getPressed(ELEVATOR_EXTEND_HIGH)) {
  //   // elevator_.ExtendHigh();
  // }
  // else{
  //   // elevator_.HoldPosition();
  // }

  // if (m_controller.getRawAxis(ELEVATOR_SET_MANUAL) > 0.75) {
  //   // elevator_.setManualVolts(m_controller.getRawAxis(ELEVATOR_RANGE));
  // }


  m_elevatorIntake.Periodic();
  m_lidar.Periodic();

  m_elevatorIntake.UpdateLidarData(m_lidar.getData());
  m_rollers.UpdateLidarData(m_lidar.getData());
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
  m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);

  // TESTING CODE
  // MAKE SURE BLUE RIGHT OR ELSE ROBOT WILL UNALIVE ITSELF
  m_autoPath.AddPoses(AutoPaths::BIG_BOY);
  m_autoPath.StartMove();
}

void Robot::AutonomousPeriodic()
{
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;
  vec::Vector2D driveVel = m_autoPath.GetVel();
  double angVel = m_autoPath.GetAngVel();

  double curYaw = m_odometry.GetAng();

  m_swerveController->SetRobotVelocity(driveVel, angVel, curYaw, deltaT);

  m_autoPath.Periodic();
  m_prevTime = curTime;
}

void Robot::TeleopInit() {
  m_swerveController->SetFeedForward(0, 1, 0);
  // m_swerveController->SetAngleCorrectionPID(SwerveConstants::ANG_CORRECT_P, SwerveConstants::ANG_CORRECT_I, SwerveConstants::ANG_CORRECT_D);
  // m_autoLineup.SetPosFF({.maxSpeed = AutoConstants::TRANS_MAXSP, .maxAccel = AutoConstants::TRANS_MAXACC});
  // m_autoLineup.SetAngFF({.maxSpeed = AutoConstants::ANG_MAXSP, .maxAccel = AutoConstants::ANG_MAXACC});
  // m_lidar.TeleopInit();
}

void Robot::TeleopPeriodic() {
  double curTime = Utils::GetCurTimeS();
  double time1 = curTime;
  double deltaT = curTime - m_prevTime;

  double lx = m_controller.getWithDeadContinuous(SWERVE_STRAFEX, 0.1);
  double ly = m_controller.getWithDeadContinuous(SWERVE_STRAFEY, 0.1);

  double rx = m_controller.getWithDeadContinuous(SWERVE_ROTATION, 0.1);

  bool fast = !m_controller.getPressed(SLOW);
  double mult = fast ? SwerveConstants::NORMAL_SWERVE_MULT : SwerveConstants::SLOW_SWERVE_MULT;
  double vx = std::clamp(lx, -1.0, 1.0) * mult;
  double vy = std::clamp(ly, -1.0, 1.0) * mult;
  double w = -std::clamp(rx, -1.0, 1.0) * mult / 2;

  vec::Vector2D curPos = m_odometry.GetPosition();
  double curYaw = m_odometry.GetAng();

  // frc::SmartDashboard::PutNumber("curYaw", curYaw);

  vec::Vector2D setVel = {-vy, -vx};

  if (!Utils::NearZero(setVel)) {
    m_isTrimming = false;
  }

  // get auto lineup pos
  int posVal = m_controller.getValue(ControllerMapData::SCORING_POS, 0);
  if (posVal) {
    m_posVal = posVal;
  }
  int heightVal = m_controller.getValue(ControllerMapData::GET_LEVEL, 0);
  if (heightVal) {
    m_heightVal = heightVal;
  }

  // frc::SmartDashboard::PutNumber("score pos val", m_posVal);
  // frc::SmartDashboard::PutNumber("Height val", m_heightVal);

  vec::Vector2D tgtPos = m_autoLineup.GetTargetPos();
  frc::SmartDashboard::PutString("cur tgt pos", tgtPos.toString());
  frc::SmartDashboard::PutNumber("cur tgt Ang", m_autoLineup.GetTargetAng());
  frc::SmartDashboard::PutBoolean("trimming", m_isTrimming);

  if (m_posVal && m_heightVal) {
    FieldConstants::ScorePair scorePair = Utils::GetScoringPos(m_posVal, m_heightVal, m_red);
    double ang = Utils::NormalizeAng(m_joystickAng + M_PI);

    vec::Vector2D scorePos = scorePair.first;
    double lidarOffset = scorePair.second;
    double lidarReading = 0;

    if (m_lidar.hasCone()) {
      lidarReading = m_lidar.getConePos() / 100.0;
      scorePos -= {0, m_red ? lidarOffset - lidarReading : lidarReading - lidarOffset};
    } else if (m_lidar.hasCube()) {
      lidarReading = m_lidar.getCubePos() / 100.0;
      scorePos += {0, m_red ? lidarOffset - lidarReading : lidarReading - lidarOffset};
    }

    // frc::SmartDashboard::PutNumber("Lidar reading", lidarReading);
    
    m_autoLineup.SetPosTarget(scorePos, false);
    m_autoLineup.SetAngTarget(ang, false);
  }

  // trim, offset shold be opposite direction of offset so it moves correct direction
  double trimX = -m_controller.getValueOnce(ControllerMapData::GET_TRIM_X, 0.0);
  double trimY = -m_controller.getValueOnce(ControllerMapData::GET_TRIM_Y, 0.0);
  vec::Vector2D offset = {trimX, trimY};
  if (m_red) {
    offset = rotate(offset, M_PI / 2);
  } else {
    offset = rotate(offset, -M_PI / 2);
  }
  if (!Utils::NearZero(offset) && m_posVal && m_heightVal && Utils::NearZero(curPos - tgtPos, AutoConstants::TRIM_DIST)) {
    m_isTrimming = true;
  }
  m_odometry.AddTrimOffset(offset);

  AutoLineup::ExecuteState curPosAutoState = m_autoLineup.GetPosExecuteState();
  AutoLineup::ExecuteState curAngAutoState = m_autoLineup.GetAngExecuteState();

  //                                          don't auto lineup to (0,0)
  if (m_controller.getPressed(AUTO_LINEUP) && m_posVal && m_heightVal) {
    // m_isAutoLineup = true;
    if (m_isTrimming) {
      // just do feedforward for trim, just needs to move a little
      m_autoLineup.SetPosPID(0, 0, 0);
    } else {
      double tkP = frc::SmartDashboard::GetNumber("trans kP", AutoConstants::TRANS_KP);
      double tkI = frc::SmartDashboard::GetNumber("trans kI", AutoConstants::TRANS_KI);
      double tkD = frc::SmartDashboard::GetNumber("trans kD", AutoConstants::TRANS_KD);
      m_autoLineup.SetPosPID(tkP, tkI, tkD);
    }

    if (curPosAutoState != AutoLineup::EXECUTING_TARGET) {
      m_autoLineup.StartPosMove();
    }
    if (curAngAutoState != AutoLineup::EXECUTING_TARGET) {
      m_autoLineup.StartAngMove();
    }

    vec::Vector2D driveVel = m_autoLineup.GetVel();
    double angVel = m_autoLineup.GetAngVel();
    // double angVel = 0;

    // frc::SmartDashboard::PutString("Auto drive vel", driveVel.toString());
    // frc::SmartDashboard::PutNumber("Auto ang vel", angVel);

    m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);
    m_swerveController->SetRobotVelocity(driveVel, angVel, curYaw, deltaT);
  } else {
    m_isAutoLineup = false;
    m_autoLineup.StopPos();
    m_autoLineup.StopAng();

    m_swerveController->SetFeedForward(0, 1, 0);
    m_swerveController->SetRobotVelocityTele(setVel, w, curYaw, deltaT, m_joystickAng);
  }

  m_autoLineup.Periodic();
  m_swerveController->Periodic();

  double time2 = Utils::GetCurTimeS();

  // frc::SmartDashboard::PutString("pos:", m_pos.toString());
  // frc::SmartDashboard::PutString("vel:", vel.toString());
  // frc::SmartDashboard::PutString("setVel:", setVel.toString());
  // frc::SmartDashboard::PutNumber("setAngVel:", w);

  if (m_controller.getTriggerDown(MANUAL1) && m_controller.getTriggerDown(MANUAL2)) {
    double elH = -m_controller.getWithDeadContinuous(ELEVATOR_H, 0.1);
    double intakeAng = -m_controller.getWithDeadContinuous(INTAKE_ANG, 0.1);
    m_elevatorIntake.ManualPeriodic(elH, intakeAng);
    // frc::SmartDashboard::PutBoolean("Manual", true);
  } else {
    // frc::SmartDashboard::PutBoolean("Manual", false);
    m_elevatorIntake.TeleopPeriodic();
    bool cone = Utils::IsCone(m_posVal);
    m_elevatorIntake.SetCone(cone);
    m_rollers.SetCone(cone);
    if(m_controller.getPressedOnce(SCORE_HIGH))
      m_elevatorIntake.ScoreHigh();
    else if (m_controller.getPressedOnce(SCORE_MID))
      m_elevatorIntake.ScoreMid();
    else if (m_controller.getPressedOnce(SCORE_LOW))
      m_elevatorIntake.ScoreLow();
    else if (m_controller.getPressedOnce(STOW))
      m_elevatorIntake.Stow();
    else if (m_controller.getPressedOnce(HP))
      m_elevatorIntake.IntakeFromHPS();
    else if (m_controller.getPressedOnce(GROUND_INTAKE))
      m_elevatorIntake.IntakeFromGround();
    else if (m_controller.getPOVDownOnce(INTAKE_FLANGE)){
      m_elevatorIntake.IntakeFlange();

    }
  }
  if (m_controller.getPressedOnce(INTAKE))
    m_rollers.Intake();
  else if (m_controller.getPressed(INTAKE))
    m_rollers.HoldIntake();
  else if (m_controller.getPressedOnce(OUTTAKE))
    m_rollers.Outtake();

  m_rollers.Periodic();
  double time3 = Utils::GetCurTimeS();

  // frc::SmartDashboard::PutNumber("swerve time", time2 - time1);
  // frc::SmartDashboard::PutNumber("el time", time3 - time2);

  // if (time2 - time1 > 0.01) {
  //   std::cout << "Swerve Time Overrun" << std::endl;
  // } else {
  //   std::cout << "El Time Overrun" << std::endl;
  // }

  m_prevTime = curTime;
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
  // m_swerveController->SetFeedForward(SwerveConstants::kS, SwerveConstants::kV, SwerveConstants::kA);

  // m_curVolts = 0;
}

void Robot::TestPeriodic() {
  // // if (m_controller.getPressedOnce(START_POS_AUTO)) {
  // //   if (m_autoLineup.GetAngExecuteState() == AutoLineup::NOT_EXECUTING) {
  // //     m_autoLineup.StartAngMove();
  // //   } else {
  // //     m_autoLineup.StopAng();
  // //   }
  // // }

  // // vec::Vector2D vel = m_autoLineup.GetVel();
  // // double angVel = m_autoLineup.GetAngVel();
  // double curYaw = m_odometry.GetAng();

  // // frc::SmartDashboard::PutString("AutoVel", vel.toString());
  // // frc::SmartDashboard::PutNumber("angVel", angVel);

  // if (m_curVolts >= 5) {
  //   m_curVolts = 5;
  // }

  // m_swerveController->SetRobotVelocity({m_curVolts, 0}, 0, curYaw, 0.02);

  // double curS = Utils::GetCurTimeS();
  // if (curS - m_prevTimeTest > 0.2) {
  //   m_prevTimeTest = curS;
  //   m_curVolts += 0.5;
  // }

  // // m_autoLineup.Periodic();
  // vec::Vector2D robotVel = m_swerveController->GetRobotVelocity(curYaw);

  // auto curTime = frc::Timer::GetFPGATimestamp();
  // auto timeus = curTime.convert<units::microsecond>(); 

  // m_voltsLog.Append(m_curVolts, timeus.value());
  // m_speedLog.Append(magn(robotVel), timeus.value());
  // m_swerveController->Periodic();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
