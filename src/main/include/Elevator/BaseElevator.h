
class BaseElevator{
    public:
        BaseElevator();
        void ExtendMid();
        void ExtendLow();
        void ExtendHigh();
        //extended position in meters (how far the elevator has extended from the point where it )
        void ExtendToCustomPos(double newPos);
        double GetPos();
        double GetVel();
};