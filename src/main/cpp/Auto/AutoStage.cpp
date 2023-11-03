#include "Auto/AutoStage.h"

/**
 * @brief Construct a new Auto Stage:: Auto Stage object
 * 
 * @param allPaths the paths for a particular auto setup to use. read from a config file.
 */
AutoStage::AutoStage(std::vector<AutoPathInit> initAllPaths, int startPathIdx) {
    for (int i = 0 ; i < initAllPaths.size(); i++){
        AutoPathInit a = initAllPaths[i];
        m_allPaths[i] = a.path;
        if (i == startPathIdx)
            continue;
        cueToPath[a.cue] = {i, &a.path};
    }
    m_startIdx = startPathIdx;
}

void AutoStage::Start(){
    m_state = IN_PROGRESS;
    StartPath({m_startIdx, &m_allPaths[m_startIdx]});
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
    if (m_curPaths.empty()){ 
        m_state == DONE;
        return;
    }
    for (auto i : m_curPaths){
        i.path->AutonomousPeriodic();
        auto itr = cueToPath.lower_bound(i.index);
        double curCompletion =  i.path->GetCompletionPercentage();
        while (itr!=cueToPath.end()){
            double curCue = itr->first;
            if (curCue <= i.index + curCompletion){
                StartPath(itr->second);
            } else break;
            itr++;
        }
        if (curCompletion >= 1.0) donePaths.push_back(i);
    }

    for (auto i : donePaths){
        m_donePaths.insert(i);
        m_curPaths.erase(i);
    }
}



void AutoStage::StartPath(AutoPathX xpath){
    if (m_donePaths.contains(xpath)) return;
    std::string type = typeid(*xpath.path).name();
    std::regex EIreg("\d+EIAutoPath"), swrvReg("\d+SwerveAutoPath");
    bool isEI = std::regex_match(type, EIreg), isSwrve = std::regex_match(type, swrvReg);
    if (isEI) {
        if (curEI) return; 
        else curEI = true;
    } if (isSwrve) {
        if (curSwrve) return; 
        else curSwrve = true;
    }

    xpath.path->Start();
    xpath.path->AutonomousPeriodic();
    m_curPaths.insert(xpath);
}