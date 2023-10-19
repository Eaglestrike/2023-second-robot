#include "Drive/Auto/AutoPaths.h"

/**
 * @brief Construct a new Auto Paths:: Auto Paths object
 * 
 * @param next_start the percentage (from 0 to 1) of completion of one path at which the next path should start.
 */
AutoPaths::AutoPaths(double next_start) {
    start_next_current_completion = next_start;
}

/**
 * @brief Returns the current execution state
 * 
 * @return AutoPaths::ExecuteState the state of the path
 */
AutoPaths::ExecuteState AutoPaths::getCurrentState() const {
    return current_state_;
}

/**
 * @brief Returns the value at which the next AutoPaths item should start.
 * 
 * @return double a double, from 0 to 1.
 */
double AutoPaths::getNextStart() {
    return start_next_current_completion;
}

/**
 * @brief Returns whether the AutoPath is still executing
 * 
 * @return true if the AutoPath is finished executing, false otherwise
 */
bool AutoPaths::isFinished() const {
    return current_state_ == ExecuteState::AT_TARGET;
}