
class BaseElevator{
    public:
        enum ElevatorState{
            EXTENDING,
            EXTENDED,
            STOWED,
            STOWING,
            KILLED
        };
        BaseElevator();
        void TeleopPeriodic();
        // void ExtendMid();
        // void ExtendLow();
        // void ExtendHigh();
        //extended position in meters (how far the elevator has extended from the point where it )
        void ExtendToCustomPos(double newPos);
        void Stow();
        ElevatorState GetState();
        double GetPos();
        double GetVel();
};