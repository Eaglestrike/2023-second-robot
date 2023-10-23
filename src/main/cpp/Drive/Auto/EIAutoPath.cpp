#include "Drive/Auto/EIAutoPath.h"

EIAutoPath::EIAutoPath(ActionToTake action, double next): AutoPaths::AutoPaths(next) {
    action_ = action;
}

double EIAutoPath::getCompletionPercentage() {
    return 0.0;
}

void EIAutoPath::periodic() {}