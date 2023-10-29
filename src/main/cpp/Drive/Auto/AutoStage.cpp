#include "Drive/Auto/AutoStage.h"
#include <cmath>
#include <algorithm>

/**
 * @brief Construct a new Auto Stage:: Auto Stage object
 * 
 * @param allPaths the paths for a particular auto setup to use. read from a config file.
 */
AutoStage::AutoStage(std::vector<AutoPath> allPaths, int startPathIdx) {
    StartPath({startPathIdx, 0.0, allPaths[startPathIdx]});
    // need to fill cueToPath
};

/**
 * @brief Called every periodic cycle. Manages the auto paths and executes them.
 * 
 */
void AutoStage::AutonomousPeriodic() {
    std::vector<AutoPathX> donePaths;
    // should error check not telling same mechanism to do smt 2x
    for (auto i : curPaths){
        i.path.AutonomousPeriodic();
        auto itr = cueToPath.lower_bound(i.index+i.lastCompletion);
        double curCompletion =  i.path.GetCompletionPercentage();
        while (true){
            double curCue = itr->first;
            if (curCue <= i.index + curCompletion){
                StartPath(itr->second);
            } else break;
            itr++;
        }
        i.lastCompletion = curCompletion;
        if (curCompletion == 1.0) donePaths.push_back(i);
    }

    for (auto i : donePaths){
        curPaths.erase(i);
    }
}

void AutoStage::Periodic(){
    EIAutoPath::Periodic();
}

void AutoStage::Init(){
    EIAutoPath::Init();
}


void AutoStage::StartPath(AutoPathX xpath){
    xpath.path.Start();
    xpath.path.AutonomousPeriodic();
    curPaths.insert(xpath);
}
