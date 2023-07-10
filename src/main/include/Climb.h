#include <Constants.h>
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>

#include <iostream>

class Climb {
 public:
  enum State {
    STOWED,
    EXTENDED,

    STOWING,
    EXTENDING,
    LIFTING, // did not include lifted bc if its at the lifted pos still needs to correct
  };

  Climb();

  void RobotInit();
  void Periodic();

  void Stow();
  void Extend();
  void Lift();

  State getState();

  void ZeroEncoder();
  void ResetPIDs();

 private:
  void changeState(State newState);
  void updatePos();
  
  WPI_TalonFX m_motor{ClimbConstants::MOTOR_ID};
  frc2::PIDController m_regPID{ClimbConstants::EXTND_STOW_P, ClimbConstants::EXTND_STOW_I,
                                   ClimbConstants::EXTND_STOW_D};
  frc2::PIDController m_climbPID{ClimbConstants::CLIMB_P, ClimbConstants::CLIMB_I,
                                   ClimbConstants::CLIMB_D};
  
  State m_state;
  double m_currentPos;
};