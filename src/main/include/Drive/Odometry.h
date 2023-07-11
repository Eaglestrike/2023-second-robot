#pragma once

#include <cstddef>
#include <map>

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Tracking posititon of robot usign scuffe` Kalman filter
 * 
 * kalman filters le x and y pos on field
 * does complementary filters angle but alpha depends on speed (literally what alex did)
 * 
 * i truly have no clue what im doing
 */
class Odometry
{
public:
  struct KalmanState {
    vec::Vector2D pos;
    vec::Vector2D vAvg;
    double ang;
    double E;
  };

  Odometry(double E0, double Q, double kAng, double k, double maxTime);

  void PredictFromWheels(vec::Vector2D vAvg, double navXAng, std::size_t curTime);
  void UpdateFromCamera(double x, double y, double angZ, std::size_t timeOffset, std::size_t curTime);

  void Reset(std::size_t curTime);
  void SetTerms(double E0, double Q, double kAng, double k, double maxTime);

  double GetEstimatedX() const;
  double GetEstimatedY() const;
  double GetEstimatedAng() const;

private:
  double m_E0;
  double m_Q;
  double m_kAng;
  double m_k;
  double m_maxTime;

  std::map<std::size_t, KalmanState> m_states;
};
