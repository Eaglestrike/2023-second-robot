#include "AutoPath.h"
#include "EIAutoPath.h"
#include <vector>
#include <unordered_set>
#include <map>

struct AutoPathInit{
    double cue;
    AutoPath& path;
};

class AutoStage {
    public:
        AutoStage(std::vector<AutoPathInit> allPaths, int startPathIdx);
        void AutonomousPeriodic();
        void Periodic();
        void Init();
        

    private:
        struct AutoPathX{
            int index;
            double lastCompletion;
            AutoPath& path;
        };
        std::vector<AutoPath> allPaths; // never changes beyond init
        std::unordered_set<AutoPathX> curPaths;
        std::map<double, AutoPathX> cueToPath; //where cue is a decimal where ones place represents index and decimal represents completion

        void StartPath(AutoPathX xpath);
};