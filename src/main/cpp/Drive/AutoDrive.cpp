#include "Drive/AutoDrive.h"

AutoDrive::AutoDrive(SwerveControl *swerveController, Odometry *odometry)
  : m_swerveController{swerveController}, m_odometry{odometry}, m_targetAng{0}, m_startTime{0},
  m_ffPos{0, 0}, m_ffAng{0, 0}, m_state{NOT_EXECUTING} {}


void AutoDrive::SetAbsTargetPose(vec::Vector2D target, double ang) {
  m_targetPos = target;
  m_targetAng = ang;
}