#include "Drive/Odometry.h"

#include <frc/smartdashboard/SmartDashboard.h>
// #include <cmath>
// #include <cstdlib>

// #include "Constants.h"
typedef vec::Vector2D Vector;

/**
 * Constrcutor
 * 
 * @param E0 Initial error value
 * @param Q wheel odometry measurement noise
 * @param kAng angle constant of proportionality for logistic function calculating trust of camera's angle measurement
 * @param k constant of proportionality for linear function calculating noise of camera xy measurement
 * @param maxTime maximum time before discarding measurements, in s
*/
Odometry::Odometry(double E0, double Q, double kAng, double k, double maxTime)
  : m_E0{E0}, m_Q{Q}, m_kAng{kAng}, m_k{k}, m_maxTime{maxTime}
{
  m_states[0].E = E0;
  m_states[0].ang = 0;
  if(useSmartDashboard){
    frc::SmartDashboard::PutNumber("Carpet Angle", m_carpetConfig.direction.angle());
    frc::SmartDashboard::PutNumber("shiftDistance", m_carpetConfig.shiftDistance);
    frc::SmartDashboard::PutNumber("shiftDistanceK", m_carpetConfig.shiftDistanceK);
    frc::SmartDashboard::PutNumber("perpShiftDistance", m_carpetConfig.perpShiftDistance);
    frc::SmartDashboard::PutNumber("perpShiftDistanceK", m_carpetConfig.perpShiftDistanceK);
  }
}

/**
 * Resets all odometry by clearing states map
 * 
 * @param curTime current robot time (from startup)
*/
void Odometry::Reset(std::size_t curTime) {
  m_states.clear();

  m_states[curTime].E = m_E0;
  m_states[curTime].ang = 0;
}

/**
 * Predicts current position and angle given wheel velocities and navx
 * 
 * @param vAvgCur average velocity from wheels, rotated
 * @param navXAng current navX angle
 * @param curTime current robot time (from startup), in ms
*/
void Odometry::PredictFromWheels(vec::Vector2D vAvgCur, double navXAng, std::size_t curTime)
{
  auto lastIt = m_states.rbegin();

  std::size_t kPrev = lastIt->first;
  Vector posPrev = lastIt->second.pos;
  Vector vAvgPrev = lastIt->second.vAvg;
  double ePrev = lastIt->second.E;
  double angPrev = lastIt->second.ang;

  //Euler's method
  double timeDiff = static_cast<double>(curTime - kPrev);
  if(timeDiff == 0){
    return;
  }
  Vector posDiff = vAvgCur * timeDiff;

  if(useSmartDashboard){
    double newAngle = frc::SmartDashboard::GetNumber("Carpet Angle", m_carpetConfig.direction.angle());
    m_carpetConfig.direction = vec::Vector2D{1, 0}.rotate(newAngle); //Where the carpet points
    m_carpetConfig.perpDirection = m_carpetConfig.direction.rotate(M_PI/2.0);
    m_carpetConfig.shiftDistance = frc::SmartDashboard::GetNumber("shiftDistance", m_carpetConfig.shiftDistance);
    m_carpetConfig.shiftDistanceK = frc::SmartDashboard::GetNumber("shiftDistanceK", m_carpetConfig.shiftDistanceK);
    m_carpetConfig.perpShiftDistance = frc::SmartDashboard::GetNumber("perpShiftDistance", m_carpetConfig.perpShiftDistance);
    m_carpetConfig.perpShiftDistanceK = frc::SmartDashboard::GetNumber("perpShiftDistanceK", m_carpetConfig.perpShiftDistanceK);
  }

  //Carpet math, different behavior depending on how the drivebase drives on the carpet
  double carpetAlignment = posDiff.dot(m_carpetConfig.direction);
  double carpetPerpAlignment = posDiff.dot(m_carpetConfig.perpDirection);
  Vector carpetAlignmentX = m_carpetConfig.direction * carpetAlignment; 
  Vector carpetAlignmentY = m_carpetConfig.perpDirection * carpetPerpAlignment;
  //Moving along = pushing back hairs
  if(carpetAlignment > 0){ //If moving in direction of carpet
    carpetAlignmentX *= m_carpetConfig.perpShiftDistanceK; //wheels are moving more, odometry excess
    if(m_lastCarpetDir.x() < 0){ //If changed from moving along carpet to not, shifts hairs back
       carpetAlignmentX += -m_carpetConfig.direction * m_carpetConfig.perpShiftDistance; //Shift odometry back against direction of hairs
    }
  }
  else if(m_lastCarpetDir.x() > 0){//If changed from moving against carpet to not, shifts hairs forward
    carpetAlignmentX += m_carpetConfig.direction * m_carpetConfig.perpShiftDistance; //Shift odometry forward with direction of hairs
  }
  //Perpendicular movement (if it exists)
  carpetAlignmentY *= m_carpetConfig.perpShiftDistanceK; //Always shifts by a factor (hair can move side to side)
  if(carpetPerpAlignment > 0){ //Check change direction
    if(m_lastCarpetDir.y() < 0){ //If changed from moving along carpet to not, shifts hairs back
      carpetAlignmentY += -m_carpetConfig.direction * m_carpetConfig.perpShiftDistance; //Shift odometry back against direction of hairs
    }
  }
  else if(m_lastCarpetDir.y() > 0){//If changed from moving against carpet to not, shifts hairs forward
    carpetAlignmentY += m_carpetConfig.direction * m_carpetConfig.perpShiftDistance; //Shift odometry forward with direction of hairs
  }
  m_lastCarpetDir = {carpetAlignment, carpetPerpAlignment};

  posDiff = carpetAlignmentX + carpetAlignmentY;

  //Store into map
  Vector pos = posPrev + posDiff;
  Vector vAvg = vAvgCur;
  double ang = navXAng;
  double E = ePrev + m_Q;

  m_states[curTime].pos = pos;
  m_states[curTime].ang = ang;
  m_states[curTime].vAvg = vAvgCur;
  m_states[curTime].E = E;
}

// /**
//  * Predicts current position and angle given wheel velocities
//  * 
//  * @param vAvg The wheel velocities, averaged
//  * @param navXAng current navX angle
//  * @param curTime current robot time (from start), in ms
// */
// void Odometry::PredictFromWheels(vec::Vector2D vAvg, double navXAng, std::size_t curTime)
// {
//   // clear matrix if >500ms ago
//   while (m_states.size() > 0 && std::abs(static_cast<long long>(curTime - m_states.begin()->first)) > m_maxTime) {
//     m_states.erase(m_states.begin());
//   }

//   // calculate change in time
//   double deltaT;
//   if (m_states.size() == 0) {
//     deltaT = curTime;
//   } else {
//     double prevTime = m_states.rbegin()->first;
//     deltaT = curTime - prevTime;
//   }

//   // update x, y, process covariance
//   double prevX;
//   double prevY;
//   Eigen::Matrix<double, 2, 2> prevP;
//   if (m_states.size() == 0) {
//     prevX = 0;
//     prevY = 0;
//     prevP = Eigen::Matrix<double, 2, 2>::Identity() * m_pInitial;
//   } else {
//     prevX = m_states.rbegin()->second.state(0, 0); 
//     prevY = m_states.rbegin()->second.state(1, 0);
//     prevP = m_states.rbegin()->second.P;  
//   }

//   auto vAvgRotate = vAvg.rotate(navXAng);
//   double curXPred = prevX + x(vAvgRotate) * deltaT;
//   double curYPred = prevY + y(vAvgRotate) * deltaT;  

//   Eigen::Matrix<double, 2, 2> Q = Eigen::Matrix<double, 2, 2>::Identity() * m_posStdDev;
//   Eigen::Matrix<double, 2, 2> curP = prevP + Q;

//   // update angle
//   double curAng = navXAng;

//   // create state
//   KalmanState curState;
//   curState.state << curXPred, curYPred;
//   curState.angle = curAng;
//   curState.vAvg = vAvg;
//   curState.P = curP;

//   std::size_t timeIndex = static_cast<std::size_t>(curTime);
//   m_states[timeIndex] = curState;
// }

// /**
//  * Updates from camera data
//  * 
//  * @param x Camera x data
//  * @param y Camera y data 
//  * @param angZ Camera angle
//  * @param timeOffset Time offset from camera read
//  * @param curTime Current time
// */
// void Odometry::UpdateFromCamera(double x, double y, double angZ, std::size_t timeOffset, std::size_t curTime)
// {
//   // don't update if camera data is greater than a certain amount of time
//   if (timeOffset > m_maxTime) {
//     return;
//   }

//   // gets state in the past
//   auto it = m_states.lower_bound(curTime - timeOffset);

//   if (it == m_states.end()) {
//     return;
//   }

//   auto pastState = it->second;

//   // calculates kalman gain
//   auto R = Eigen::Matrix<double, 2, 2>::Identity() * m_measurementStdDev;
//   auto gain = pastState.P * (pastState.P + R).inverse();

//   // updates state and covariance
//   Eigen::Matrix<double, 2, 1> measure;
//   measure << x, y;
//   auto newState = pastState.state + gain * (measure - pastState.state);
//   auto newP = (Eigen::Matrix<double, 2, 2>::Identity() - gain) * pastState.P;

//   // calculate trustworthiness of camera 
//   double curVel = magn(pastState.vAvg);
//   double alpha = 0.1 / (1 + std::exp(10 * (curVel - 0.2)));
//   double newTheta = alpha * angZ + (1 - alpha) * pastState.angle;
//   double addVal = newTheta - pastState.angle - m_angOffset;
//   m_angOffset += addVal;

//   // update values from this snapshot in time + 1 to present
//   KalmanState curState;
//   for (auto it2 = it; it2 != m_states.end(); it2++) {
//     m_states[timeIndex] = curState;

//     // update x, y, process covariance
//     double prevX;
//     double prevY;
//     Eigen::Matrix<double, 2, 2> prevP;
//     if (m_states.size() == 0) {
//       prevX = 0;
//       prevY = 0;
//       prevP = Eigen::Matrix<double, 2, 2>::Identity() * m_pInitial;
//     } else {
//       prevX = it2->second.state(0, 0); 
//       prevY = it2->second.state(1, 0);
//       prevP = it2->second.P;  
//     }

//     auto vAvgRotate = vAvg.rotate(navXAng);
//     double curXPred = prevX + x(vAvgRotate) * deltaT;
//     double curYPred = prevY + y(vAvgRotate) * deltaT;  

//     Eigen::Matrix<double, 2, 2> Q = Eigen::Matrix<double, 2, 2>::Identity() * m_posStdDev;
//     Eigen::Matrix<double, 2, 2> curP = prevP + Q;

//     // create state
//     curState.state << curXPred, curYPred;
//     curState.angle = curAng;
//     curState.vAvg = vAvg;
//     curState.P = curP;
//   }

//   it++;
//   // delete values from begin() to current time
//   if (m_states.size() > 0) {
//     m_states.erase(m_states.begin(), it);
//   }
// }

// /**
//  * Resets odometry and past values
// */
// void Odometry::Reset()
// {
//   m_states.clear();
//   m_angOffset = 0;
// }

// /**
//  * Sets covariance error, recommended to reset after
//  * 
//  * @param posStdDev position standard deviation
//  * @param measurementStdDev measurement standrad deviation
//  * @param pInitial inital value of covariance matrix
//  * @param maxTime Max time before ignore/discard vlaues
// */
// void Odometry::SetErrorTerms(double posStdDev, double measurementStdDev, double pInitial, double maxTime)
// {
//   m_posStdDev = posStdDev;
//   m_measurementStdDev = measurementStdDev;
//   m_pInitial = pInitial;
//   m_maxTime = maxTime;
// }

// /**
//  * 
// */