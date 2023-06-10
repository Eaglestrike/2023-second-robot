#pragma once

#include <cstddef>
#include <map>

#include "thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Tracking posititon of robot usign scuffe` Kalman filter
 * 
 * kalman filters le x and y pos on field
 * does complementary filters angle but alpha depends on speed (literally what alex did)
 * 
 * i truly have no clue what im doing
 * 
 * If odometry randomly resets to 0, it's probably because maxTime is too small
 * or you are not calling PredictFromWheels every periodic.
 */
class KalmanFilter
{
public:
  struct KalmanState {
    vec::Vector2D pos;
    vec::Vector2D vAvg;
    double ang;
    double E;
  };

  KalmanFilter(double E0, double Q, double kAng, double k, double maxTime);

  void PredictFromWheels(vec::Vector2D vAvg, double navXAng, std::size_t curTime);
  void UpdateFromCamera(vec::Vector2D pos, double angZ, std::size_t timeOffset, std::size_t curTime);

  void Reset(std::size_t curTime);
  void SetTerms(double E0, double Q, double kAng, double k, double maxTime);

  vec::Vector2D GetEstimatedPos() const;
  double GetEstimatedAng() const;

private:
  double m_E0;
  double m_Q;
  double m_kAng;
  double m_k;
  double m_maxTime;

  // maps time from startup in ms (stored as integer) to Kalman state
  std::map<std::size_t, KalmanState> m_states;
};
