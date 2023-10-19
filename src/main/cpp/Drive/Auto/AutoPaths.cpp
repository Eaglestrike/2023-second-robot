#include "Drive/Auto/AutoPaths.h"

AutoPaths::AutoPaths(double next_start) {
    start_next_current_completion = next_start;
}

AutoPaths::ExecuteState AutoPaths::getCurrentState() {
    return current_state_;
}