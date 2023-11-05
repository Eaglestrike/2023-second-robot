#pragma once


#include "Auto/BaseAuto.h"

class ThreePiece : public BaseAuto{
    using ElevatorTarget = ElevatorConstants::ElevatorTarget;
    using enum ElevatorTarget;

    //Position of pieces during auto (blue)
    const double PIECE_X = 7.068;
    const double PIECE_Y_1 = 0.919;
    const double PIECE_Y_2 = 2.138;
    const double PIECE_Y_3 = 3.358;
    const double PIECE_Y_4 = 4.577;
    
    public:
        enum State{
            NONE,
            GOING_FIRST,
            FIRST_PLACE,
            GOING_TO_SECOND,
            PICKING_SECOND,
            COMING_FROM_SECOND,
            SECOND_PLACE,
            GOING_TO_THIRD,
            PICKING_THIRD,
            COMING_FROM_THIRD,
            THIRD_PLACE,
            GOING_OUT
        };

        void setSetup(bool firstCone, bool secondCone, bool thirdCone);
        void setTarget(ElevatorTarget firstTarget, ElevatorTarget secondTarget, ElevatorTarget thirdTarget);

        void Init() override;
        void Periodic() override;

        vec::Vector2D GetDriveVel() override;
        double GetAngVel() override;
        bool DockNow() override;

    private:
        bool m_hasCalcPositions;
        void CalcPositions(); //Calculates the target positions

        double m_autoStartTime;

        struct PieceSetup{
            bool firstCone = true; //Preloaded piece
            bool secondCone = false; //Outer piece
            bool thirdCone = false; //Inner piece
        } m_setup;

        struct TargetSetup{  
            ElevatorConstants::ElevatorTarget firstTarget = HIGH;
            ElevatorConstants::ElevatorTarget secondTarget = HIGH;
            ElevatorConstants::ElevatorTarget thirdTarget = MID;
        } m_targets;

        bool m_grid[3][3]; // [cone/cube/cone][low/mid/high]

        State m_state = NONE;
        State m_prevState = NONE;
        double m_stateStartTime;
};
