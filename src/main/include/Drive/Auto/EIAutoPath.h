#include "AutoPaths.h"

class EIAutoPath: public AutoPaths {
    public:
        // note: these were added from elevator,
        // assuming that intake will react to them.
        enum ActionToTake {
            STOW,
            EXTEND_MID,
            EXTEND_LOW,
            EXTEND_HIGH,
            HOLD_POSITION,
        };


        EIAutoPath(ActionToTake action, double next);

    private:
        ActionToTake action_;

};