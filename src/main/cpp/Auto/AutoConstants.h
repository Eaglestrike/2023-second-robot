
#include "Auto/AutoStage.h"
#include "Auto/SwerveAutoPath.h"
#include "Auto/EIAutoPath.h"
#include "Auto/AutoPath.h"
#include "Elevator/ElevatorIntake.h"

#include <vector>

namespace AutoConstants {
    EIAutoPath E_HIGH{ElevatorIntake::HIGH, true};
    EIAutoPath E_LOW{ElevatorIntake::LOW, true};
    EIAutoPath E_MID{ElevatorIntake::MID, true};

    AutoPaths::SwervePose swp_1 = {0.0, 10, 20, 30, 30, 40, 10};
    AutoPaths::SwervePose swp_2 = {0.0, 10, 20, 30, 30, 40, 10};
    AutoPaths::SwervePose swp_3 = {0.0, 10, 20, 30, 30, 40, 10};

    // used for initializing the SwerveAutoPath
    std::vector<AutoPaths::SwervePose> swerve_poses = {
        swp_1, swp_2, swp_3
    };


    std::vector<AutoPathInit> auto_path_vector = {
        {0.0, &E_LOW},
        {1.0, &E_MID},
        {1.0, &E_HIGH},
        {1.5, &E_STOW},
    };

}