#include "AutoPaths.h"

class EIAutoPath: public AutoPaths {
    public:
        enum ActionToTake {
            STOW,
            EXTEND_MID,
            EXTEND_LOW,
            EXTEND_HIGH,
            HOLD_POSITION,
        };
};