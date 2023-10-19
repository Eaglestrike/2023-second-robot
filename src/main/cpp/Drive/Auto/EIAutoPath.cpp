#include "Drive/Auto/EIAutoPath.h"

EIAutoPath::EIAutoPath(ActionToTake action, double next): AutoPaths::AutoPaths(next) {
    action_ = action;
}