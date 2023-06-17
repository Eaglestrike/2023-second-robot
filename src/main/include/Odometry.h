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
  Odometry(std::shared_ptr<SwerveControl> swerveController, std::shared_ptr<AHRS> navx);
  
  void SetStartPos(vec::Vector2D startPos);
  void SetStartAng(double startAng);
  void CalcStart(std::size_t fieldPos);
  void SetStart(vec::Vector2D startPos, double startAng);
  void SetKFTerms(double E0, double Q, double kAng, double k, double maxTime);
  void SetCamData(vec::Vector2D camPos, double camAng, double angNavX, std::size_t camID, std::size_t howLongAgo);
  void Reset();

  vec::Vector2D GetPosition() const;
  double GetAng() const;

  void Periodic();

private:
  vec::Vector2D m_startPos;
  double m_startAng;
  vec::Vector2D m_curPos;
  double m_curAng;

  std::shared_ptr<SwerveControl> m_swerveController;
  std::shared_ptr<AHRS> m_navx;
  KalmanFilter m_filter;
};