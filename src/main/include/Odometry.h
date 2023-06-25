#pragma once

#include <cstddef>
#include <memory>

#include <AHRS.h>

#include "KalmanFilter.h"
#include "SwerveControl.h"
#include "thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Gets information about robot's velocity, position, and heading
*/
class Odometry {
public:
  Odometry() = delete;
  Odometry(SwerveControl *swerveController, AHRS *navx);
  
  void SetStartPos(vec::Vector2D startPos);
  void SetStartAng(double startAng);
  void SetStart(vec::Vector2D startPos, double startAng);
  void SetKFTerms(double E0, double Q, double kAng, double k, double maxTime);
  void SetCamData(vec::Vector2D camPos, double camAng, double angNavX, std::size_t camID, std::size_t howLongAgo);
  void Reset();

  vec::Vector2D GetPosition() const;
  double GetAng() const;

  void Periodic();

private:
  vec::Vector2D m_startPos; // position offset
  double m_startAng;
  vec::Vector2D m_curPos;
  double m_curAng;

  SwerveControl *m_swerveController;
  AHRS *m_navx;
  KalmanFilter m_filter;
};