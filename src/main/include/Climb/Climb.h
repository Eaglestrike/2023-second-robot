#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
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

  using AccelerationV = units::compound_unit<units::radians_per_second, units::inverse<units::second>>;
  using Acceleration = units::compound_unit<units::angular_velocity::radians_per_second, units::inverse<units::time::seconds>>; 
  using kv_unit = units::compound_unit<units::volts, units::inverse<units::radians_per_second>>;
  using ka_unit = units::compound_unit<units::volts, units::inverse<AccelerationV>>;

 private:
  void ChangeState(State newState);
  void UpdatePos();
  void CollectTuningData();
  void UpdateVelAcc();
  void UpdateTargetPosVel();
  double StepsToRad(double steps);
  
  WPI_TalonFX m_motor{ClimbConstants::MOTOR_ID};
  frc2::PIDController m_regPID{ClimbConstants::EXTND_STOW_P, ClimbConstants::EXTND_STOW_I,
                                   ClimbConstants::EXTND_STOW_D};
  frc2::PIDController m_climbPID{ClimbConstants::CLIMB_P, ClimbConstants::CLIMB_I,
                                   ClimbConstants::CLIMB_D};
  
  frc::ArmFeedforward m_climbFF {units::volt_t(ClimbConstants::FF_S), // volts
                                units::volt_t(ClimbConstants::FF_G), // volts
                                units::unit_t<kv_unit>(ClimbConstants::FF_V), //volts*seconds/rad
                                units::unit_t<ka_unit>(ClimbConstants::FF_A)}; //volts*seconds^2/rad
  
  State m_state = State::STOWED;

  double m_currentPos; //in radians 
  double m_currentVel; //in rad/sec
  double m_currentAcc; //in rad/sec^2

  // to collect tuning data for feedforward
  std::vector<double> vel, volts; 
  double vlts =0.0;

  double m_targetPos;
  double m_targetVel;

  double m_posTurnPt;

  bool dbg = true; //to enable/disable smartdashboard stuff

};