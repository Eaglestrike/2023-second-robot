#pragma once

#include "AutoStage.h"

#include <vector>
#include <string>

class AutoManager {
    public:
        AutoManager(std::vector<AutoStage> stages): m_stages{stages}{}
        void AutonomousPeriodic(){
            if (m_curStage == -1 || m_curStage > m_stages.size()) return;
            m_stages[m_curStage].AutonomousPeriodic();
            if (m_stages[m_curStage].GetState() == AutoStage::DONE) m_curStage++;
        };

        void Start(){
            m_curStage = 0;
        };

    private:
        int m_curStage = -1;
        std::vector<AutoStage> m_stages;
};