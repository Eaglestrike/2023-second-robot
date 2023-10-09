#pragma once

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Autodocking
 * 
 * Assumes autodocking from opposite side of drivers
*/
class AutoDock {
public:
  enum State {
    NOT_DOCKING,
    PRE_DOCK,
    TOUCH_STN,
    ON_STN,
    DOCKED
  };

  AutoDock(bool isRed = false);

  State GetState() const;
  vec::Vector2D GetVel() const;

  void UpdateOdom(double r, double p, double y);
  void SetkTilt(double kTilt);
  void SetPreDockSpeed(double preDockSpeed);
  void SetMaxDockSpeed(double maxDockSpeed);
  void SetPreDockAng(double preDockAng);
  void SetDockAng(double dockAng);
  void SetDockedTol(double dockedTol);
  void SetSide(bool isRed);
  void Start();
  void Reset();
  void Periodic();

private:
  State m_curState;

  bool m_isRed;

  double m_kTilt;
  double m_preDockSpeed;
  double m_maxDockSpeed;

  double m_preDockAng;
  double m_dockAng;
  double m_dockedTol;

  double m_roll, m_pitch, m_yaw;
  vec::Vector2D m_outputVel;

  double GetTilt() const;
};