#pragma once

#include <frc/controller/ArmFeedforward.h>

class FeedForwardPID{
    public:
        using AccelerationV = units::compound_unit<units::radians_per_second, units::inverse<units::second>>;
        using Acceleration = units::compound_unit<units::angular_velocity::radians_per_second, units::inverse<units::time::seconds>>; 
        using kv_unit = units::compound_unit<units::volts, units::inverse<units::radians_per_second>>;
        using ka_unit = units::compound_unit<units::volts, units::inverse<AccelerationV>>;

        FeedForwardPID(double MAX_VEL, double MAX_ACC): MAX_VEL{MAX_VEL},  MAX_ACC{MAX_ACC}{}

        void SetFeedForwardConsts(double ks, double kg, double kv, double ka){
           m_s = ks; // volts
                                m_g = kg; // volts
                               m_v = kv; //volts*seconds/rad
                                m_a = ka; //volts*seconds^2/rad}
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
            double newTargVel;

            if (m_targetPos >= m_velTurnPt){
                newTargVel = m_targetVel - MAX_ACC*0.02;
            } else {
                newTargVel = m_targetVel + MAX_ACC*0.02; 
            }

            if (m_velTurnPt > 0)
                newTargVel = std::clamp(newTargVel, 0.0, MAX_VEL); 
            else 
                newTargVel = std::clamp(newTargVel, -MAX_VEL, 0.0); 
        
            if (newTargVel>m_targetVel)
                m_targetAcc = MAX_ACC;
            else if (newTargVel == m_targetVel)
                m_targetAcc = 0;
            else m_targetAcc = -MAX_ACC;
            m_targetVel = newTargVel;
            m_targetPos += m_targetVel*0.02;
            frc::SmartDashboard::PutNumber("target pos", m_targetPos);
            frc::SmartDashboard::PutNumber("target vel", m_targetVel);
            frc::SmartDashboard::PutNumber("target acc", m_targetAcc);

        } 

        // Returns sum of pid and feedforward control
        double Calculate(double curPos, double curVel){
            double ff = m_s*m_targetPos + m_g + m_v*m_targetVel+m_a*m_targetAcc,
            posErr = m_targetPos - curPos,
            velErr = m_targetVel - curVel;
            frc::SmartDashboard::PutNumber("position err", posErr);
            frc::SmartDashboard::PutNumber("velocity err", velErr);

            frc::SmartDashboard::PutNumber("ff out", ff);
            auto pidOut =  posErr * m_kp + velErr* m_kd;
            frc::SmartDashboard::PutNumber("pd out", pidOut);
            UpdateTargetVelAndPos();
            return ff +pidOut;
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
        double m_targetPos, m_targetVel, m_targetAcc; // in rad, rad/sec, rad/sec^2
        double m_velTurnPt; // the position at which the velocity begins to decrease in the trapexoidal motion profile
        
        //pid constants
        double m_kp, m_kd;
        double m_s, m_g, m_v, m_a;
        frc::ArmFeedforward m_ff; 
};