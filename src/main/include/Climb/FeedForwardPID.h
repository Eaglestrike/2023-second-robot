#pragma once

#include <frc/controller/ArmFeedforward.h>

class FeedForwardPID{
    public:
        using AccelerationV = units::compound_unit<units::radians_per_second, units::inverse<units::second>>;
        using Acceleration = units::compound_unit<units::angular_velocity::radians_per_second, units::inverse<units::time::seconds>>; 
        using kv_unit = units::compound_unit<units::volts, units::inverse<units::radians_per_second>>;
        using ka_unit = units::compound_unit<units::volts, units::inverse<AccelerationV>>;

        FeedForwardPID(double MAX_VEL, double MAX_ACC): MAX_ACC(MAX_ACC), MAX_VEL(MAX_VEL){}

        void SetFeedForwardConsts(double ks, double kg, double kv, double ka){
            m_ff = frc::ArmFeedforward{units::volt_t(ks), // volts
                                units::volt_t(kg), // volts
                                units::unit_t<kv_unit>(kv), //volts*seconds/rad
                                units::unit_t<ka_unit>(ka)}; //volts*seconds^2/rad}
        }
        
        void SetPIDConsts(double kp, double kd){
            m_kp = kp; m_kd = kd;
        }

        void SetTolerance(double posTolerance, double velTolerance){
           m_posTolerance = posTolerance;
           m_velTolerance = velTolerance;
        }

        // Assuming is called 1x every TeleopPeriodic call so called every 20 ms
        void UpdateTargetVelAndPos(){
            if (m_targetPos >= m_velTurnPt){
                m_targetVel = std::clamp(m_targetVel - MAX_ACC*0.02, -MAX_VEL, MAX_VEL); 
            } else {
                m_targetVel = std::clamp(m_targetVel + MAX_ACC*0.02, -MAX_VEL, MAX_VEL); 
            }
            m_targetPos += m_targetVel*0.02;
        } 

        // Returns sum of pid and feedforward control
        double Calculate(double curPos, double curVel, double curAcc){
            auto s = units::radian_t(curPos); // rads
            auto v = units::radians_per_second_t(curVel); // rad/sec
            auto a = units::unit_t<Acceleration>(curAcc); // rad/sec^2
            double ff = double(m_ff.Calculate(s,v,a)),
            posErr = m_targetPos - curPos,
            velErr = m_targetVel - curVel;
            return ff + posErr * m_kp + velErr* m_kd;
        }

        void SetSetpoint(double setpt, double curPos){
            m_setPoint = setpt;
            m_targetPos = curPos;
            m_targetVel = 0.0;
            CalcTurnPoint(curPos);
        }

        bool AtSetPoint(double curPos){
            return abs(curPos - m_setPoint) < m_posTolerance;
        }

    private:
        // calculates what Pos should be when the velocity begins decreasing 
        void CalcTurnPoint(double curPos){
            if (curPos > m_targetPos)
                m_velTurnPt = MAX_VEL*(MAX_VEL - 2.0)/(MAX_ACC*2) + m_setPoint;
            else 
                m_velTurnPt = -MAX_VEL*(MAX_VEL - 2.0)/(MAX_ACC*2) + m_setPoint;
        }

        double MAX_VEL, MAX_ACC; // volts
        double m_setPoint;
        double m_posTolerance, m_velTolerance;
        double m_targetPos, m_targetVel; // in rad and rad/sec
        double m_velTurnPt; // the position at which the velocity begins to decrease in the trapexoidal motion profile
        
        //pid constants
        double m_kp, m_kd;
        frc::ArmFeedforward m_ff; 
};