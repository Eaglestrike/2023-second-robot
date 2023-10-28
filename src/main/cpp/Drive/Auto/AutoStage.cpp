#include "Drive/Auto/AutoStage.h"
#include <cmath>
#include <algorithm>

/**
 * @brief Construct a new Auto Stage:: Auto Stage object
 * 
 * @param paths_to_use the paths for a particular auto setup to use. read from a config file.
 */
AutoStage::AutoStage(std::vector<AutoPaths> paths_to_use) {};

/**
 * @brief Called every periodic cycle. Manages the auto paths and executes them.
 * 
 */
void AutoStage::periodic() {
    if (all_paths.empty()) {
        return;
    }

    // Adds first element from all paths array to execution array
    if (paths_being_executed.empty()) {
        transferIntoCurrentExecutionVector();
    }

    // Execute all paths in the execution array
    for (int i = 0; i < paths_being_executed.size(); i++) {
        // TODO: double check this use of auto
        paths_being_executed[i].periodic();

        // If it's close enough to the correct percentage, start the next action
        if (abs(paths_being_executed[i].getNextStart() - paths_being_executed[i].getCompletionPercentage()) < COMPLETION_TOLERANCE) {
            transferIntoCurrentExecutionVector();
        }

        // remove it if it is completed
        if (paths_being_executed[i].isFinished()) {
            paths_being_executed.erase(paths_being_executed.begin() + i);
            i--;
        }
    }
}

/**
 * @brief Takes the first element from the all paths array and moves it into the current execution array
 * 
 */
void AutoStage::transferIntoCurrentExecutionVector() {
    // downcast
    // paths_being_executed.push_back(all_paths.front());
    // all_paths.erase(all_paths.begin());
}