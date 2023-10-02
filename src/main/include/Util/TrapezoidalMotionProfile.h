#include <cmath>
#include "Util/Mathutil.h"

class TrapezoidalMotionProfile {
    public:
    TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC, double curPos, double setPt);
    TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC);
    double GetVelocity() const;
    double GetPosition() const;
    double GetAcceleration() const;
    bool AtSetPoint() const;
    double GetMaxVel() const;
    double GetMaxAcc() const;

    void SetSetpoint(double curPos, double setPoint);
    void SetMaxVel(double maxVel);
    void SetMaxAcc(double maxAcc);
    void Periodic();
    private:
        // void CalcTurnTime(double curPos, double setPt);
        void CalcVelTurnPos(double curPos, double setPt);
        double m_targetPos, m_targetVel, m_targetAcc;
        double m_maxVel, m_maxAcc;
        double m_setPt;
        double m_velTurnPos;
        double m_curTime = -1;
        bool m_negProfile;
        //        m_turnTime, // the time the profile should start deaccelerating if pos, or accelerating if neg
        //        m_endTime;
};