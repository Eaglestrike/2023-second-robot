
// #include "Auto/AutoManager.h"
// #include "Auto/AutoConstants.h"

// AutoManager::AutoManager(SwerveControl& db, ElevatorIntake& ei): drivebase_(db), elevator_intake_(ei) {
//     // TODO: add items from constants file to vector, as appropriate

//     SwerveAutoPath swp_tot{AutoConstants::swerve_poses, &drivebase_};

//     std::vector<AutoPathInit> auto_path_vector = {
//         {0.0, &AutoConstants::E_LOW},
//         {1.0, &AutoConstants::E_MID},
//         {1.0, &AutoConstants::E_HIGH},
//         {1.5, &swp_tot}
//     };

//     AutoStage as{auto_path_vector, 0};

//     all_stages.push_back(as);
    
//     AutoStageX asx{"testing for everything", all_stages[0]};
//     chosenStage = asx;
//     chosenStage.stage.Init();
// };

// /**
//  * @brief Choooses which auto stage the driver wants to use
//  * 
//  * @param index idk
//  */
// void AutoManager::chooseAutoStage(int index) {
//     // initialize the chosen auto stage here
//     chosenStage = all_stages[index];

//     // read from constants file
//     chosenStage.stage.Init(elevator_intake_);
// }

// void AutoManager::CorePeriodic() {
//     // TODO: check changes against shuffleboard, then change chosenStage appropriately
// }

// void AutoManager::CoreAutonomousPeriodic() {
//     chosenStage.stage.AutonomousPeriodic();
// }