
class AutoPaths {
    public:
        enum ExecuteState {
            NOT_EXECUTING,
            EXECUTING_PATH,
            AT_TARGET
        };

        AutoPaths(double next_start);

        void periodic();
        ExecuteState getCurrentState();
        double getCompletionPercentage();

    protected:
        ExecuteState current_state_ = NOT_EXECUTING;
        double start_next_current_completion;
};