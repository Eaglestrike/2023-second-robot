#include "Auto/AutoStage.h"

/**
 * @brief Construct a new Auto Stage:: Auto Stage object
 * 
 * @param allPaths the paths for a particular auto setup to use. read from a config file.
 */
AutoStage::AutoStage(std::vector<AutoPathInit> initAllPaths, int startPathIdx) {
    for (int i = 0 ; i < initAllPaths.size(); i++){
        AutoPathInit a = initAllPaths[i];
        allPaths[i] = a.path;
        if (i == startPathIdx)
            continue;
        cueToPath[a.cue] = {i, 0.0, &a.path};
    }
    m_startIdx = startPathIdx;
}

void AutoStage::Start(){
    m_state = IN_PROGRESS;
    StartPath({m_startIdx, 0.0, &allPaths[m_startIdx]});
}

AutoStage::StageState AutoStage::GetState(){
    return m_state;
}

/**
 * @brief Called every periodic cycle. Manages the auto paths and executes them.
 * 
 */
void AutoStage::AutonomousPeriodic() {
    if (m_state == NOT_STARTED || m_state == DONE) return;
    std::vector<AutoPathX> donePaths;
    // should error check not telling same mechanism to do smt 2x
    if (curPaths.empty()){ 
        m_state == DONE;
        return;
    }
    for (auto i : curPaths){
        i.path->AutonomousPeriodic();
        auto itr = cueToPath.lower_bound(i.index+i.lastCompletion);
        double curCompletion =  i.path->GetCompletionPercentage();
        while (true){
            double curCue = itr->first;
            if (curCue <= i.index + curCompletion){
                StartPath(itr->second);
            } else break;
            itr++;
        }
        i.lastCompletion = curCompletion;
        if (curCompletion >= 1.0) donePaths.push_back(i);
    }

    for (auto i : donePaths){
        curPaths.erase(i);
    }
}

// void AutoStage::Periodic(){
//     EIAutoPath::Periodic();
// }


void AutoStage::StartPath(AutoPathX xpath){
    xpath.path->Start();
    xpath.path->AutonomousPeriodic();
    curPaths.insert(xpath);
}

void AutoStage::Initialize(std::vector<AutoPathInit> initAllPaths, int startPathIdx) {
    for (int i = 0 ; i < initAllPaths.size(); i++){
        AutoPathInit a = initAllPaths[i];
        allPaths[i] = a.path;
        if (i == startPathIdx)
            continue;
        cueToPath[a.cue] = {i, 0.0, &a.path};
    }
    StartPath({startPathIdx, 0.0, &allPaths[startPathIdx]});
}