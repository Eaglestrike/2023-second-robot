#include "Auto/AutoStage.h"

/**
 * @brief Construct a new Auto Stage:: Auto Stage object
 * 
 * @param allPaths the paths for a particular auto setup to use. read from a config file.
 */
AutoStage::AutoStage(std::vector<AutoPathInit> initAllPaths, int startPathIdx) {
    for (int i = 0 ; i < initAllPaths.size(); i++){
        AutoPathInit a = initAllPaths[i];
        m_allPaths[i] = &a.path;
        if (i == startPathIdx)
            continue;
        m_cueToPath[a.cue] = {i, &a.path};
    }
    m_startIdx = startPathIdx;
    m_mechInUse = *(new std::vector<bool>(AutoPath::LAST, false));
}

AutoStage::AutoStage(){
    m_state = NOT_STARTED;
}

void AutoStage::EnableDBG(bool d){
    dbg = d;
}

void AutoStage::Start(){
    if (m_allPaths.size() < 1) 
    m_state = IN_PROGRESS;
    StartPath({m_startIdx, m_allPaths[m_startIdx]});
}

AutoStage::StageState AutoStage::GetState(){
    return m_state;
}

/**
 * @brief Called every periodic cycle. Manages the auto paths and executes them.
 * 
 */
void AutoStage::AutonomousPeriodic() {
    if (dbg) frc::SmartDashboard::PutNumber("stage state", m_state);
    if (m_state == NOT_STARTED || m_state == DONE) return;
    std::vector<AutoPathX> donePaths;
    if (m_curPaths.empty()){ 
        m_state = DONE;
        return;
    }
    if (dbg) {
        //print curpaths
        std::vector<std::string> s;
        for (auto i : m_curPaths){
            s.push_back(i.path->toString());
        }
        std::span<std::string> spn(s);
        frc::SmartDashboard::PutStringArray("cur paths", spn);
        
        //print mech in use
        std::vector<int> k;
        for (auto i : m_mechInUse){
            k.push_back(i);
        }
        std::span<int> bSpn(k);
        frc::SmartDashboard::PutBooleanArray("mech in use", bSpn);
    }

    for (auto i : m_curPaths){
        i.path->AutonomousPeriodic();
        auto itr = m_cueToPath.lower_bound(i.index);
        double curCompletion =  i.path->GetCompletionPercentage();
        while (itr!=m_cueToPath.end()){
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
        m_mechInUse[i.path->GetType()] = false;
        m_curPaths.erase(i);
    }
}

void AutoStage::StartPath(AutoPathX xpath){
    if (m_donePaths.contains(xpath)) return;
    int tp =xpath.path->GetType();
    if (m_mechInUse[tp]) return;
    else m_mechInUse[tp] = true;
    if(dbg) std::cout << "Starting path " << xpath.index << ": " << xpath.path->toString()<< std::endl;
    xpath.path->Start();
    xpath.path->AutonomousPeriodic();
    m_curPaths.insert(xpath);
}