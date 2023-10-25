#include "Drive/Auto/AutoStage.h"
#include <cmath>
#include <algorithm>

/**
 * @brief Construct a new Auto Stage:: Auto Stage object
 * 
 * @param paths_to_use the paths for a particular auto setup to use. read from a config file.
 */
AutoStage::AutoStage(std::vector<AutoPaths> paths_to_use): all_paths(paths_to_use) {};

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
        auto& p = paths_being_executed[i];
        p.Periodic();

        // If it's close enough to the correct percentage, start the next action
        // if (abs(p.getNextStart() - p.getCompletionPercentage()) < COMPLETION_TOLERANCE) {
        //     transferIntoCurrentExecutionVector();
        // }
    }

    // Removes all elements that are finished
    auto new_end = std::remove_if(paths_being_executed.begin(), paths_being_executed.end(),
                                  [](const AutoPaths& p) { return p.GetCompletionPercentage() == 1.0; });

    paths_being_executed.erase(new_end, paths_being_executed.end());

}

/**
 * @brief Takes the first element from the all paths array and moves it into the current execution array
 * 
 */
void AutoStage::transferIntoCurrentExecutionVector() {
    auto last_element = all_paths.front();
    paths_being_executed.push_back(last_element);
    all_paths.erase(all_paths.begin());
}