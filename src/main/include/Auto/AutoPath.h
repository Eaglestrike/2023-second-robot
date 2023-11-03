#pragma once

class AutoPath {
    public:
        virtual void AutonomousPeriodic() = 0;

        virtual double GetCompletionPercentage() const{
            return m_completion;
        };
 
        virtual bool GetStarted() const{
            return m_started;
        };

        virtual void Start(){
            m_started = true;
        };

    protected:
        bool m_started = false;
        double m_completion = 0.0;
};