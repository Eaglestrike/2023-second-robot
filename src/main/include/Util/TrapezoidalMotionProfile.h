#include <cmath>
#include "Util/Mathutil.h"

class TrapezoidalMotionProfile {
    public:
    TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC, double curPos, double setPt);
    TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC);
    void SetSetpoint(double curPos, double setPoint);
    void Calculate();
    bool AtSetPoint();
    
    private:
        void CalcTurnTime(double curPos, double setPt);
        double m_targetPos, m_targetVel, m_targetAcc;
        double m_maxVel, m_maxAcc;
        double m_setPt;
        double m_curTime,
               m_turnTime, // the time the profile should start deaccelerating if pos, or accelerating if neg
               m_endTime;
};