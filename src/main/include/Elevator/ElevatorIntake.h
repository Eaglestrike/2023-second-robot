#pragma once
#include "Intake.h"
#include "BaseElevator.h"

class ElevatorIntake{
    public:
        ElevatorIntake();
        void MoveToCustomPos(double xoff, double yoff, double zoff);
        void ScoreHigh(double xoff, double yoff);
        void ScoreMid(double xoff, double yoff);
        void ScoreLow(double xoff, double yoff);
        void IntakeFromHPStation();
        void IntakeFromGround();
    private:
        Intake m_intake();
        BaseElevator m_elevator();
};