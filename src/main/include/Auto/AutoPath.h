#pragma once
#include <string>

class AutoPath {
    public:
        enum ChildType{
            EI,
            SWERVE,
            LAST
        };
        
        // AutoPath() {};

        virtual void AutonomousPeriodic() = 0;

        virtual std::string toString() = 0;

        virtual double GetCompletionPercentage() const{
            return m_completion;
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
        bool m_started = false;
        ChildType m_type; // should be intialized in constructor of childclass
        double m_completion = 0.0;
};