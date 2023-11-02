#pragma once

#include <cstddef>
#include <map>
#include <vector>

#include "Util/Mathutil.h"
#include "Util/thirdparty/hermite.hpp"
#include "Util/thirdparty/simplevectors.hpp"
#include "Drive/DriveConstants.h"
#include "Auto/AutoPath.h"
#include "Drive/SwerveControl.h"

namespace hm = hermite;
namespace vec = svector;

class SwerveAutoPath: public AutoPath {
  typedef hm::Hermite<1> Hermite1;
  typedef hm::Pose<1> Pose1;
  typedef hm::Hermite<2> Hermite2;
  typedef hm::Pose<2> Pose2;

public:
  enum ExecuteState {
    NOT_EXECUTING,
    EXECUTING_PATH,
    AT_TARGET
  };

  SwerveAutoPath(SwerveControl& drivebase, std::vector<AutoPaths::SwervePose> poses);

  void AddPose(AutoPaths::SwervePose pose);
  void AddPoses(std::vector<AutoPaths::SwervePose> poses);
  void ResetPath();
  void SetPosPID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);
  void Stop();
  void StartMove();
  void UpdateOdom(vec::Vector2D curPos, double curAng);
  void Periodic();

  vec::Vector2D GetVel() const;
  double GetAngVel() const;
  ExecuteState GetExecuteState() const;

private:
  vec::Vector2D m_curPos;
  double m_curAng;
  int m_multiplier; // markplier???

  vec::Vector2D m_curVel;
  double m_curAngVel;

  double m_startTime;
  double m_expectFinish;
  double m_prevTime;
  double m_prevTimeOdom;

  // distance managing
  double current_distance;
  double total_distance;

  // Hermite3 m_calc;
  Hermite2 m_calcTrans;
  Hermite1 m_calcAng;

  ExecuteState m_curState;

  // for position PID
  vec::Vector2D m_prevPosErr;
  vec::Vector2D m_totalPosErr;

  double m_prevAngErr;
  double m_totalAngErr;

  double m_kPPos;
  double m_kIPos;
  double m_kDPos;

  double m_kPAng;
  double m_kIAng;
  double m_kDAng;

  vec::Vector2D GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos); 
  double GetPIDAng(double deltaT, double curExpectedAng);
  bool AtTarget() const;
  bool AtTransTarget(double posErrTol, double velErrTol) const;
  bool AtRotTarget(double posErrTol, double velErrTol) const;
  double GetMultipliedAng() const;

  // methods from AutoPath, for overriding purposes
  void AutonomousPeriodic() override;
  SwerveControl& drivebase_;

  void calculateTotalDistance();
  void calculateCurrentProgress();
};