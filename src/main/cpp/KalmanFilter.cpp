#include "KalmanFilter.h"

#include <cmath>

/**
 * Constrcutor
 * 
 * @param E0 Initial error value
 * @param Q wheel odometry measurement noise
 * @param kAng angle constant of proportionality for logistic function calculating trust of camera's angle measurement
 * @param k constant of proportionality for linear function calculating noise of camera xy measurement
 * @param maxTime maximum time before discarding measurements, in s
*/
KalmanFilter::KalmanFilter(double E0, double Q, double kAng, double k, double maxTime)
  : m_E0{E0}, m_Q{Q}, m_kAng{kAng}, m_k{k}, m_maxTime{maxTime}
{
  // we want to make sure there's at least one value in the map at all times
  m_states[0].E = E0;
  m_states[0].ang = 0;
}

/**
 * Resets all odometry by clearing states map
 * 
 * @param curTime current robot time (from startup), in ms
*/
void KalmanFilter::Reset(std::size_t curTime) {
  m_states.clear();

  // add value; we want to make sure there's at least one value in the map at all times
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
void KalmanFilter::PredictFromWheels(vec::Vector2D vAvgCur, double navXAng, std::size_t curTime)
{
  // get previous state variables
  auto lastIt = m_states.rbegin();
  std::size_t kPrev = lastIt->first;
  auto posPrev = lastIt->second.pos;
  auto vAvgPrev = lastIt->second.vAvg;
  double ePrev = lastIt->second.E;

  double timeDiff = static_cast<double>(curTime - kPrev) / 1000.0;

  // position is previous position + time difference
  auto pos = posPrev + vAvgCur * timeDiff;
  auto vAvg = vAvgCur;
  double ang = navXAng;
  double E = ePrev + m_Q; // error covariance amplifies

  // assign values to map
  m_states[curTime].pos = pos;
  m_states[curTime].ang = ang;
  m_states[curTime].vAvg = vAvgCur;
  m_states[curTime].E = E;

  // delete map values > maxTime seconds ago, unless it's the last value in the map
  auto firstIt = m_states.lower_bound(curTime - static_cast<std::size_t>(m_maxTime * 1000.0)); // first iterator to include
  m_states.erase(m_states.begin(), firstIt);

  // if empty, insert zeroes
  // we want to make sure there is at least one value at all times
  if (m_states.size() == 0) {
    m_states[curTime].E = m_E0;
    m_states[curTime].ang = 0;
  }
  // this will mess up odometry but at least no runtime errors
}

/**
 * Updates data from camera
 * 
 * @param pos Robot's position relative to the field from the camera
 * @param angZ Angle reading from camera
 * @param timeOffset Difference in time between now and the time that the camera reading was read, in ms
 * @param curTime current robot time (from startup), in ms
*/
void KalmanFilter::UpdateFromCamera(vec::Vector2D pos, double angZ, std::size_t timeOffset, std::size_t curTime) {
  // if > maxTime before, ignore
  if (static_cast<double>(curTime - timeOffset) > m_maxTime * 1000.0) {
    return;
  }

  std::size_t measurementTime = curTime - timeOffset; // time of camera measurement, in ms

  // state nearest to measurement time
  auto measurementIt = m_states.lower_bound(measurementTime);
  // make sure iterator exists
  if (measurementIt == m_states.end()) {
    return;
  }

  // get predicted state variables
  auto posPred = measurementIt->second.pos;
  auto vAvgPred = measurementIt->second.vAvg;
  auto ePred = measurementIt->second.E;
  auto angPred = measurementIt->second.ang;

  // corrects position
  double camNoise = m_k * vec::magn(vAvgPred); // cam noise is proportional to robot velocity
  double kalmanGain = ePred / (ePred + camNoise); // calculates kalman gain
  vec::Vector2D correctedPos = posPred + (pos - posPred) * kalmanGain; // corrects position
  double e = (1 - kalmanGain) / ePred; // corrects error

  // corrects angle
  double alpha = 0.1 / (1 + std::exp(m_kAng * (vec::magn(vAvgPred) - 0.5))); // how much to trust angle from camera
  double ang = alpha * angZ + (1 - alpha) * angPred;
  double angDiff = ang - angPred;

  // previous states and times
  KalmanState prevState; // used to store previous state of timestep
  prevState.ang = ang;
  prevState.pos = correctedPos;
  prevState.E = e;
  prevState.vAvg = vAvgPred;
  std::size_t prevTime = measurementTime; // used to store previous time

  measurementIt++;

  // updates states from measurementTime to map end
  for (auto it = measurementIt; it != m_states.end(); it++) {
    // time difference
    double timeDiff = static_cast<double>(it->first - prevTime) / 1000.0;

    // average velocity of current time step
    auto vAvgCur = it->second.vAvg;

    // new position is previous position + time difference
    auto pos = prevState.pos + vAvgCur * timeDiff;
    auto vAvg = vAvgCur;
    double ang = it->second.ang + angDiff;
    double E = prevState.E + m_Q; // error covariance amplifies

    // assign values to state
    prevState.pos = pos;
    prevState.ang = ang;
    prevState.vAvg = vAvg;
    prevState.E = E;
    prevTime = it->first;

    // updates state, then prevState is used for next iteration
    it->second = prevState;
  }

  // delete map values > maxTime seconds ago, unless it's the last value in the map
  auto firstIt = m_states.lower_bound(curTime - static_cast<std::size_t>(m_maxTime * 1000.0)); // first iterator to include
  m_states.erase(m_states.begin(), firstIt);

  // if empty, insert zeroes
  // we want to make sure there is at least one value at all times
  if (m_states.size() == 0) {
    m_states[curTime].E = m_E0;
    m_states[curTime].ang = 0;
  }
  // this will mess up odometry but at least no runtime errors
}

/**
 * Sets Kalman Filter terms
 * 
 * It is recommended to reset odometry after setting terms
 * 
 * All of these must be > 0 or things will break
 * 
 * @param E0 initial error covariance
 * @param Q noise of wheels
 * @param kAng angle constant in logistic function higher = lower trust in camera for higher velocities
 * @param k constant of proportionality between speed and camera noise
 * @param maxTime max time before ignore, in s
*/
void KalmanFilter::SetTerms(double E0, double Q, double kAng, double k, double maxTime) {
  m_E0 = E0;
  m_Q = Q;
  m_kAng = kAng;
  m_k = k;
  m_maxTime = maxTime;
}

/**
 * Gets estimated position
 * 
 * @returns Estimated position
*/
vec::Vector2D KalmanFilter::GetEstimatedPos() const {
  auto itLatest = m_states.rbegin();
  return itLatest->second.pos;
}

/**
 * Gets estimated angle
 * 
 * @returns Estimated angle
*/
double KalmanFilter::GetEstimatedAng() const {
  auto itLatest = m_states.rbegin();
  return itLatest->second.ang;
}