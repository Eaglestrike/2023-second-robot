#include "Auto/SwerveAutoPath.h"

#include "Drive/DriveConstants.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

/**
 * Constructor
*/
SwerveAutoPath::SwerveAutoPath(SwerveControl& drivebase, std::vector<AutoPaths::SwervePose> poses) : 
  m_curAng{0}, m_multiplier{0}, m_curAngVel{0}, m_prevTime{0}, m_prevTimeOdom{0},
  m_calcTrans{100}, m_calcAng{100}, m_curState{NOT_EXECUTING},
  m_kPPos{0}, m_kIPos{0}, m_kDPos{0}, m_kPAng{0}, m_kIAng{0}, m_kDAng{0}, drivebase_{drivebase}
{
  using namespace AutoConstants;
  SetPosPID(TRANS_KP, TRANS_KI, TRANS_KD);
  SetAngPID(ANG_KP, ANG_KI, ANG_KD);
  calculateTotalDistance();
  AddPoses(poses);
}

/**
 * @brief Recalculates the total distance that will be travelled by this spline
 */
void SwerveAutoPath::calculateTotalDistance() {
  total_distance = m_calcTrans.getMaxDistance(0.1);
}

/**
 * @brief Calculates the current progress, by averaging the x, y, and angle components.
 */
void SwerveAutoPath::calculateCurrentProgress() {
  double num_waypoints = m_calcTrans.getAllWaypoints().size();

  double curr_x = m_curPos.x();
  double curr_y = m_curPos.y();

  double num_stages_completed = 0;
  double num_stages_total = num_waypoints * 2;

  for (SwerveAutoPath::Pose2 waypoint: m_calcTrans.getAllWaypoints()) {
    double dim_1 = waypoint.getPos().at(0);
    double dim_2 = waypoint.getPos().at(1);

    if (curr_x > dim_1) {
      num_stages_completed++;
    }

    if (curr_y > dim_2) {
      num_stages_completed++;
    }
  }

  // double, representing the range of values
  // for example, if there are 2 waypoints, the possible completion values are: [0.25, 0.5, 0.75, 1.0]
  double current_distance = m_calcTrans.getLength(current_distance);
}

/**
 * Adds a singular pose
 * 
 * @param pose Pose to add, an SwerveAutoPath::SwervePose object (use initializer list)
 * 
 * @note Robot will not take shortest path between angles, to make it take the shortest path,
 * add or subtract M_PI to angle
 * @note Units are m, rad, and s
*/
void SwerveAutoPath::AddPose(AutoPaths::SwervePose pose) {
  Pose2 poseTrans = {pose.time, {pose.x, pose.y}, {pose.vx, pose.vy}};
  Pose1 poseAng = {pose.time, {pose.ang}, {pose.angVel}};

  m_calcTrans.insertOrReplace(poseTrans);
  m_calcAng.insertOrReplace(poseAng);
}

/**
 * Adds a list of poses
 * 
 * @param poses Poses to add, a vector of SwerveAutoPath::SwervePose objects (use initializer list)
 * 
 * @note Robot will not take shortest path between angles, to make it take the shortest path,
 * add or subtract M_PI to angle
 * @note Units are m, rad, and s
*/
void SwerveAutoPath::AddPoses(std::vector<AutoPaths::SwervePose> poses) {
  for (auto pose : poses) {
    AddPose(pose);
  }
}

/**
 * Clears auto paths
*/
void SwerveAutoPath::ResetPath() {
  m_calcTrans = Hermite2{100};
  m_calcAng = Hermite1{100};
}


/**
 * Sets position correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void SwerveAutoPath::SetPosPID(double kP, double kI, double kD) {
  m_kPPos = kP;
  m_kIPos = kI;
  m_kDPos = kD;
}

/**
 * Sets angular position correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void SwerveAutoPath::SetAngPID(double kP, double kI, double kD) {
  m_kPAng = kP;
  m_kIAng = kI;
  m_kDAng = kD;
}

/**
 * Stops auto path
*/
void SwerveAutoPath::Stop() {
  m_curState = NOT_EXECUTING;
}

/**
 * Starts auto path
*/
void SwerveAutoPath::StartMove() {
  m_curState = EXECUTING_PATH;
  m_startTime = Utils::GetCurTimeS();
  m_expectFinish = m_calcTrans.getHighestTime();
}

/**
 * Updates odometry
 * 
 * @note Make sure this is called every cycle
 * 
 * @param pos Current position
 * @param ang Current angle
*/
void SwerveAutoPath::UpdateOdom(vec::Vector2D curPos, double curAng) {
  m_curPos = curPos;

  // update multiplier of angle
  double curTimeS = Utils::GetCurTimeS();
  double curAngSpeed = (curAng - m_curAng) / (curTimeS - m_prevTimeOdom);
  if (curAngSpeed < -AutoConstants::UNREASONABLE_ANG_SPEED) {
    // looping around counterclockwise
    m_multiplier++;
  } else if (curAngSpeed > AutoConstants::UNREASONABLE_ANG_SPEED) {
    // looping around clockwise
    m_multiplier--;
  }
  m_prevTimeOdom = curTimeS;

  m_curAng = curAng;
}

/**
 * Periodic function
*/
void SwerveAutoPath::Periodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_curState) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      m_curAngVel = 0;
      break;
    case EXECUTING_PATH:
    {
      double relTime = curTime - m_startTime;
      double highestTime = m_calcTrans.getHighestTime();
      double getTime = (relTime <= highestTime ? relTime : highestTime);

      // TODO: manage the completion code
      double current_distance = m_calcAng.getLength(getTime);

      vec::Vector2D curExpectedPos = m_calcTrans.getPos(getTime);
      vec::Vector2D curExpectedVel = m_calcTrans.getVel(getTime);

      vec::Vector2D correctionVel = GetPIDTrans(deltaT, curExpectedPos);
      vec::Vector2D totalVel = curExpectedVel + correctionVel;

      double curExpectedAng = m_calcAng.getPos(getTime)[0];
      double curExpectedAngVel = m_calcAng.getVel(getTime)[0];

      double correctionAngVel = GetPIDAng(deltaT, curExpectedAng); 
      double totalAngVel = curExpectedAngVel + correctionAngVel;

      m_curVel = totalVel;
      m_curAngVel = totalAngVel;

      if (AtTarget()) {
        m_curState = AT_TARGET;
      }

      // TODO impelemnt angle
      break;
    }
    case AT_TARGET:
    {
      m_curVel = {0, 0};
      m_curAngVel = 0;
      m_completion = 1.0;

      if (!AtTarget()) {
        m_curState = NOT_EXECUTING;
      }

      break;
    }
  }

  m_prevTime = curTime;
}

/**
 * Returns whether robot  is at the target
 * 
 * Assumes autoconstant error tolerance
 * 
 * @returns Wheter robot is where it is supposed to be 
*/
bool SwerveAutoPath::AtTarget() const {
  using namespace AutoConstants;
  return AtTransTarget(TRANS_POS_ERR_TOLERANCE, TRANS_VEL_ERR_TOLERANCE)
    && AtRotTarget(ANG_POS_ERR_TOLERANCE, ANG_VEL_ERR_TOLERANCE);
}

/**
 * Returns whether robot is at the target translationally
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Whether robot is where it is supposed to be 
*/
bool SwerveAutoPath::AtTransTarget(double posErrTol, double velErrTol) const {
  vec::Vector2D targetPos = m_calcTrans.getPos(m_calcTrans.getHighestTime());

  return Utils::NearZero(targetPos - m_curPos, posErrTol) && Utils::NearZero(m_curVel, velErrTol);
}

/**
 * Returns whether robot is at the target rotationally
 * 
 * @param posErrTol position error tolerance
 * @param velErrTol velocity error tolerance
 * 
 * @returns Whether robot is where it is supposed to be 
*/
bool SwerveAutoPath::AtRotTarget(double posErrTol, double velErrTol) const {
  double targetAng = m_calcAng.getPos(m_calcAng.getHighestTime())[0];
  return Utils::NearZero(targetAng - GetMultipliedAng(), posErrTol) && Utils::NearZero(m_curAngVel, velErrTol);
}

/**
 * Gets angle with multiplier (angle represented as something greater than 180 or smaller than -180)
 * 
 * @returns Multiplied angle
*/
double SwerveAutoPath::GetMultipliedAng() const {
  return m_curAng + m_multiplier * M_PI * 2;
}

/**
 * Calculates position PID for trnaslational motion
 * 
 * @param deltaT time difference
 * @param curExpectedPos current expected translational motion
 * 
 * @returns Velocity from PID
*/
vec::Vector2D SwerveAutoPath::GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos) {
  vec::Vector2D err = curExpectedPos - m_curPos;

  vec::Vector2D deltaErr = (err - m_prevPosErr) / deltaT;
  m_totalPosErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalPosErr * m_kIPos + deltaErr * m_kDPos;

  m_prevPosErr = err;

  return res;
}

/**
 * Calculates position PID for trnaslational motion
 * 
 * @param deltaT time difference
 * @param curExpectedPos current expected translational motion
 * 
 * @returns Velocity from PID
*/
double SwerveAutoPath::GetPIDAng(double deltaT, double curExpectedAng) {
  double curAngAbs = m_curAng + m_multiplier * M_PI * 2;
  double err = curExpectedAng - curAngAbs;

  double deltaErr = (err - m_prevAngErr) / deltaT;
  m_totalAngErr += err * deltaT;
  double res = err * m_kPAng + m_totalAngErr * m_kIPos + deltaErr * m_kDPos;

  m_prevAngErr = err;

  return res;
}

/**
 * Gets current translational velocity
 * 
 * @returns Translational velocity
*/
vec::Vector2D SwerveAutoPath::GetVel() const {
  return m_curVel;
}

/**
 * Gets current rotational velocity
 * 
 * @returns Rotational velocity
*/
double SwerveAutoPath::GetAngVel() const {
  // TODO add angle calculations
  return m_curAngVel;
}

/**
 * Gets current execute state
 * 
 * @returns Current execute state
*/
SwerveAutoPath::ExecuteState SwerveAutoPath::GetExecuteState() const {
  return m_curState;
}

/**
 * @brief Called every periodic cycle. Manages the motors.
 * 
 */
void SwerveAutoPath::AutonomousPeriodic() {
  // does the calculations
  Periodic();
  
  // implements it
  drivebase_.SetRobotVelocity(m_curVel, m_curAngVel, m_curAng, m_prevTime);
}