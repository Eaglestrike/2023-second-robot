#include "Elevator/Intake/Rollers.h"

#include "Util/Mathutil.h"

void Rollers::SetCone(bool cone) {
    m_cone = cone;
}

void Rollers::UpdateLidarData(LidarReader::LidarData lidarData) {
    if (lidarData.isValid) m_hasGamePiece = lidarData.hasCone || lidarData.hasCube;
}

void Rollers::Intake() {
    if (m_state == INTAKE) {
        Stop();
    } else {
        m_state = INTAKE;
    }
}

void Rollers::Outtake() {
    if (m_state == OUTTAKE) {
        Stop();
    } else {
        m_state = OUTTAKE;
    }
}

void Rollers::Stop() {
    m_state = STOP;
}

void Rollers::Periodic() {
    // cone out = postiive, in = negative
    double intakeVolt = m_cone ? -IntakeConstants::CONE_INFO.IN_VOLTS : IntakeConstants::CUBE_INFO.IN_VOLTS;
    double outtakeVolt = m_cone ? IntakeConstants::CONE_INFO.OUT_VOLTS : -IntakeConstants::CUBE_INFO.OUT_VOLTS;
    double keepVolt = m_cone ? IntakeConstants::CONE_INFO.KEEP_VOLTS : IntakeConstants::CUBE_INFO.KEEP_VOLTS;

    double setVolts = 0;

    switch (m_state) {
        case INTAKE: {
            if (m_hasGamePiece) {
                if (Utils::GetCurTimeS() - m_hasGamePieceStart > 1.0) {
                    m_state = RETAIN;
                }
            } else {
                m_hasGamePieceStart = Utils::GetCurTimeS();
            }
            setVolts = intakeVolt;
            break;
        }
        case RETAIN:
            if (!m_hasGamePiece) {
                m_state = INTAKE;
            }
            setVolts = keepVolt;
            break;
        case OUTTAKE:
            setVolts = outtakeVolt;
            break;
        case STOP:
            setVolts = 0;
            break;
        default:
            setVolts = 0;
    }

    m_rollerMotor.SetVoltage(units::volt_t{setVolts});
}