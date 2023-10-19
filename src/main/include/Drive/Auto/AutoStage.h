#include "AutoPaths.h"
#include <vector>

class AutoStage {
    public:
        AutoStage(std::vector<AutoPaths> paths_to_use);
        void periodic();

    private:
        std::vector<AutoPaths> all_paths;
        std::vector<AutoPaths> paths_being_executed;
};