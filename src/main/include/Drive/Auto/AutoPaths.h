
class AutoPaths {
    public:
        enum ExecuteState {
            NOT_EXECUTING,
            EXECUTING_PATH,
            AT_TARGET
        };

        AutoPaths(double next_start);

        virtual void periodic() = 0;
        ExecuteState getCurrentState();
        virtual double getCompletionPercentage() = 0;

    protected:
        ExecuteState current_state_ = NOT_EXECUTING;
        double start_next_current_completion;
};