
class AutoPaths {
    public:
        enum ExecuteState {
            NOT_EXECUTING,
            EXECUTING_PATH,
            AT_TARGET
        };

        AutoPaths(double next_start);

        // Every subclass is responsible for managing their state in the periodic method.
        virtual void periodic() = 0;

        // TODO: rename this for clarity
        double getNextStart();
        virtual double getCompletionPercentage() = 0;
        bool isFinished() const;
        ExecuteState getCurrentState() const;

    protected:
        ExecuteState current_state_ = NOT_EXECUTING;
        double start_next_current_completion;
};