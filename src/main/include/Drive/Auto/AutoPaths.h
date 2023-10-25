
class AutoPaths {
    public:
        virtual void Periodic() = 0;
        virtual void Init() = 0;
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