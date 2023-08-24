#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "FeedForwardPID.h"
#include "ClimbConstants.h"

class Climb {
 public:
  enum State {
    STOWED,
    EXTENDED,
    STOWING, 
    EXTENDING, 
    LIFTING, // did not include lifted because while climbing robot should never stop correcting 
             // since the final "lifted" position is not sustainable unlike the stowed and extended positions
};

  Climb();

  void RobotInit();
  void TeleopPeriodic();

  void Stow();
  void Extend();
  void Lift();

  State GetState();

  void ZeroEncoder();
  void ResetPIDs();

 private:
  void ChangeState(State newState);
  void UpdatePos();
  void UpdateVelAcc();
  double StepsToRad(double steps);

  //for tuning and debug
  void CollectTuningData();
  std::string StateToString();
  
  WPI_TalonFX m_motor{ClimbConstants::MOTOR_ID};
  frc2::PIDController m_regPID{ClimbConstants::EXTND_STOW_P, ClimbConstants::EXTND_STOW_I,
                                   ClimbConstants::EXTND_STOW_D};
  frc2::PIDController m_climbPID{ClimbConstants::CLIMB_P, ClimbConstants::CLIMB_I,
                                   ClimbConstants::CLIMB_D};
  FeedForwardPID m_FFPID{ClimbConstants::MAX_VEL, ClimbConstants::MAX_ACC}; // feed forward PID for climb
  
  State m_state = State::STOWED;

  double m_currentPos; //in radians 
  double m_currentVel; //in rad/sec
  double m_currentAcc; //in rad/sec^2

  // to collect tuning data for feedforward
  std::vector<double> vel, volts; 
  double vlts = 0.0;

  bool dbg = ClimbConstants::SMART_DASH; //to enable/disable smartdashboard stuff

};