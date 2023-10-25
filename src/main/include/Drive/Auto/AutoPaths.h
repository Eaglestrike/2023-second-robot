
class AutoPaths {
    public:
        virtual void Periodic() = 0;
        virtual void Init() = 0;
        virtual void AutonomousPeriodic() = 0;

        virtual const double GetCompletionPercentage(){
            return m_completion;
        };
        virtual const bool GetStarted(){
            return m_started;
        };
        virtual void Start(){
            m_started = true;
        };

    protected:
        bool m_started = false;
        double m_completion = 0.0;
};