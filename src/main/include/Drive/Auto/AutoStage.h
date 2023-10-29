#include "AutoPath.h"
#include "EIAutoPath.h"
#include <vector>
#include <set>
#include <map>

class AutoStage {
    public:
        struct AutoPathX{
            int index;
            double lastCompletion;
            AutoPath& path;
        };
        AutoStage(std::vector<AutoPath> allPaths, int startPathIdx);
        void Periodic();
        void Init();
        void AutonomousPeriodic();


    private:
        std::vector<AutoPath> allPaths; // never changes beyond init
        std::set<AutoPathX> curPaths;
        std::map<double, AutoPathX> cueToPath; //where cue is a decimal where ones place represents index and decimal represents completion

        void StartPath(AutoPathX xpath);
};