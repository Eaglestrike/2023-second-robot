#pragma once

#include "Util/Mechanism.h"
#include "Drive/SwerveControl.h"
#include "Elevator/ElevatorIntake.h"
#include "AutoStage.h"

#include <vector>
#include <string>

class AutoManager: public Mechanism {
    public:
        AutoManager(SwerveControl& db, ElevatorIntake& ei);
        void chooseAutoStage(int index);

    private:
        struct AutoStageX {
            std::string name; // for driver choosing

            // TODO: fix this compilation
            AutoStage::AutoStage stage;
        }

        std::vector<AutoStageX> all_stages;

        void loadAutoStage();

        virtual void CorePeriodic() override;
        virtual void CoreAutonomousInit() override;
        virtual void CoreAutonomousPeriodic() override;

        // member variables
        SwerveControl& drivebase_;
        ElevatorIntake& elevator_intake_;

        AutoStageX chosenStage;
};