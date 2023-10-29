
class AutoPath {
    public:
        // virtual static void Periodic() = 0;
        // virtual void Init() = 0;
        virtual void AutonomousPeriodic() = 0;

        AutoPath(double cue): m_cue(cue){};

        virtual double GetCompletionPercentage() const{
            return m_completion;
        };
        // virtual double GetLastCompletionPercentage() const{
        //     return m_lastCompletion;
        // };
        virtual bool GetStarted() const{
            return m_started;
        };
        virtual bool GetCue() const{
            return m_cue;
        };
        virtual void Start(){
            m_started = true;
        };

    protected:
        bool m_started = false;
        double m_completion = 0.0;
        // double m_lastCompletion = 0.0;
        double m_cue;
};