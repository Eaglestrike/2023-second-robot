#pragma once

#include <cmath>
#include <algorithm>
#include "AutoPath.h"
#include "EIAutoPath.h"
#include <vector>
#include <iostream>
#include <set>
#include <map>

struct AutoPathInit{
    double cue;
    AutoPath& path;
    std::string toString(){
        return (std::to_string(cue) + ": " + path.toString());
    }
};

class AutoStage {
    public:
        enum StageState{
            NOT_STARTED,
            IN_PROGRESS,
            DONE
        };
        AutoStage();
        AutoStage(std::vector<AutoPathInit> initAllPaths, int startPathIdx);
        void Start();
        StageState GetState();
        void AutonomousPeriodic();
        void EnableDBG(bool d = true);
        

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
        bool dbg = false; 
        StageState m_state = NOT_STARTED;
        std::vector<AutoPath*> m_allPaths; // never changes beyond init
        std::set<AutoPathX> m_curPaths;
        std::set<AutoPathX> m_donePaths;
        std::map<double, AutoPathX> m_cueToPath; //where cue is a decimal where ones place represents index and decimal represents completion
        std::vector<bool> m_mechInUse{false, false};

        void StartPath(AutoPathX xpath);
};