#pragma once
#include <string>

class AutoPath {
    public:
        enum ChildType{
            EI,
            SWERVE,
            LAST
        };
        AutoPath(){};
        AutoPath(double compTol): m_competionTolerance{compTol}{};

        virtual void AutonomousPeriodic() = 0;

        virtual std::string toString() = 0;

        virtual double GetCompletionPercentage() const{
            return (m_completion>1-m_competionTolerance)? 1.0 : m_completion;
        };
 
        virtual bool GetStarted() const{
            return m_started;
        };
         
        virtual bool GetType() const{
            return m_type;
        };

        virtual void Start(){
            m_started = true;
        };

    protected:
        double m_competionTolerance = 0.0;
        bool m_started = false;
        ChildType m_type; // should be intialized in constructor of childclass
        double m_completion = 0.0;
};