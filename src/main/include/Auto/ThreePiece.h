#pragma once

#include "ShuffleboardSender/ShuffleboardSender.h"

#include "Auto/BaseAuto.h"

class ThreePiece : public BaseAuto{
    using ElevatorTarget = ElevatorConstants::ElevatorTarget;
    using enum ElevatorTarget;
    using SwervePose = AutoPaths::SwervePose;

    //Position of pieces during auto (blue)
    const double PIECE_X = 7.068;
    const double PIECE_Y_1 = 0.919;
    const double PIECE_Y_2 = 2.138;
    const double PIECE_Y_3 = 3.358;
    const double PIECE_Y_4 = 4.577;
    
    //Used for navigation around
    const double CHARGE_X = 3.826 + 1.0; //Center + width of charge station
    const double CHARGE_Y = 2.748;
    const double NAV_WIDTH = 2.0;//Distance from center of charge station to go around
    const double NAV_VEL = 3.0;

    public:
        enum State{
            NONE,
            ALIGN_FIRST,
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

        ThreePiece(ElevatorIntake &ei, AutoLineup &al, AutoPath &ap, Rollers &r);

        void setSetup(bool firstCone, bool secondCone, bool thirdCone);
        void setTarget(ElevatorTarget firstTarget, ElevatorTarget secondTarget, ElevatorTarget thirdTarget);

        void Init() override;
        void Periodic() override;

        vec::Vector2D GetDriveVel() override;
        double GetAngVel() override;
        bool DockNow() override;

    private:
        void startNewPath(std::vector<SwervePose> poses);
        void ScorePos(bool cone, ElevatorTarget target);

        //Calculating targets
        void CalcPositions(); //Calculates the target positions
        SwervePose CalcScorePositions(bool cone, ElevatorTarget target, bool coneSpots[3], int posOffset);

        //Structs for setup
        struct PieceSetup{
            bool firstCone = true; //Preloaded piece
            bool secondCone = false; //Outer piece
            bool thirdCone = false; //Inner piece
        } m_setup;

        struct TargetSetup{  
            ElevatorTarget first = HIGH;
            ElevatorTarget second = HIGH;
            ElevatorTarget third = MID;
        } m_targetHeights;

        //State logic
        double m_autoStartTime;

        State m_state = NONE;
        State m_prevState = NONE;
        double m_stateStartTime;
        SwervePose m_targetPose;

        struct SwerveTargets{
            SwervePose placingFirst;
            SwervePose navChargeForward; //Moving past the charge station
            SwervePose pickingSecond;
            SwervePose navChargeBack; //Moving back from charge station
            SwervePose placingSecond;
            SwervePose pickingThird; //Reuse navigating charge from second for third
            SwervePose placingThird;
        }m_targetPoses;

        ShuffleboardSender shuff_;
};
