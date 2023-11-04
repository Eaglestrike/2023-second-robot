#pragma once

#include "Util/Mathutil.h"

namespace EIAutoConstants{
    const double HALFSTOW_PERCENT = 0.25; // 0 to 0.25 are halfstowing
    const double ELEVATOR_PERCENT = 0.5; // 2.5 to 0.75 are moving elevator
    const double INTAKE_PERCENT = 0.25; // 0.75 to 1 are moving wrist
    const double COMPLETION_TOLERANCE = 0.0;
}

namespace SwerveAutoConstants {
    const double COMPLETION_TOLERANCE = 0.0;
    const double SWERVE_TOLERANCE = 1.0;
    // AutoPaths::SwervePose swp_1 = {0.0, 0.5, 0.5, 0.5, 0.5, 0, 0};
    // AutoPaths::SwervePose swp_2 = {1.0, 1.0, 1.0, 0.6, 0.6, 0, 0};
    // AutoPaths::SwervePose swp_3 = {2.0, 10, 20, 30, 30, 40, 10};
    

    // used for initializing the SwerveAutoPath
    // const std::vector<AutoPaths::SwervePose> swerve_poses = {
    //     {0.0, 0.5, 0.5, 0.5, 0.0, 0.0}, {1.0, 1.0, 1.0, 0.6, 0.6, 0, 0}
    // };
}