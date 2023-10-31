#pragma once

#include <cmath>
#include <algorithm>
#include "AutoPath.h"
#include "EIAutoPath.h"
#include <vector>
#include <set>
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
        void Init(ElevatorIntake& ei);
        

    private:
        struct AutoPathX{
            int index;
            double lastCompletion;
            AutoPath* path;
            bool operator<(const AutoPathX &other) const {
                return index < other.index;
            }
            bool operator==(const AutoPathX &other) const {
                return index == other.index;
            }
            bool operator !=(const AutoPathX &other) const {
                return index != other.index;
            }
        };
        std::vector<AutoPath> allPaths; // never changes beyond init
        std::set<AutoPathX> curPaths;
        std::map<double, AutoPathX> cueToPath; //where cue is a decimal where ones place represents index and decimal represents completion

        void StartPath(AutoPathX xpath);
};