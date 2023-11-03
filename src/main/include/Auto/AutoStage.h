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
        enum StageState{
            NOT_STARTED,
            IN_PROGRESS,
            DONE
        };
        AutoStage();
        void Start();
        StageState GetState();
        AutoStage(std::vector<AutoPathInit> allPaths, int startPathIdx);
        void AutonomousPeriodic();
        

    private:
        struct AutoPathX{
            int index;
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
        int m_startIdx;
        StageState m_state = NOT_STARTED;
        std::vector<AutoPath> m_allPaths; // never changes beyond init
        std::set<AutoPathX> m_curPaths;
        std::set<AutoPathX> m_donePaths;
        std::map<double, AutoPathX> cueToPath; //where cue is a decimal where ones place represents index and decimal represents completion

        void StartPath(AutoPathX xpath);
};