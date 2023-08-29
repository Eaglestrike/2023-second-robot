//
// Created by Vir Shah on 6/14/23.
//

#include "Elevator.h"
#include "frc/controller/ElevatorFeedforward.h"

// debug getters
double Elevator::getElevatorHeight() {
    return (left_.GetSelectedSensorPosition() + right_.GetSelectedSensorPosition()) / 2.0;
}

double Elevator::getLeftRotation() {
    return left_.GetSelectedSensorPosition();
}

double Elevator::getRightRotation() {
    return right_.GetSelectedSensorPosition();
}

/**
 * @brief Construct a new Elevator:: Elevator object
 * 
 * @param leftID the ID for the left motor
 * @param rightID  the ID for the right motor
 */
Elevator::Elevator(int leftID, int rightID):
    left_(leftID, "canbus"),
    right_(rightID, "canbus"),
    feedforward_(
        ElevatorConstants::KS,
        ElevatorConstants::KV,
        ElevatorConstants::KA,
        ElevatorConstants::KG,
        ElevatorConstants::KP,
        ElevatorConstants::KD,
        ElevatorConstants::MAX_ELEVATOR_EXTENSION
        ) 
{

};

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::periodic() {

    if (current_state == STOPPED) {
        return;
    }

    std::pair<double, double> current_values;
    current_values.first = left_.GetSelectedSensorVelocity();
    current_values.second = (left_.GetSelectedSensorPosition() / SwerveConstants::TALON_FX_COUNTS_PER_REV) * ElevatorConstants::ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED;

    double motor_output = feedforward_.periodic(current_values);

    // sketch
    if (current_state == MOVING_TO_DOCKED) {
        motor_output *= -1.0;
    }

    left_.SetVoltage(units::volt_t{motor_output});
    right_.SetVoltage(-1.0 * units::volt_t{motor_output});
}

/**
 * This will be used to set the next position that the elevator should move to 
 * @param new_pos the next state that the elevator should be in
 */
void Elevator::setState(Elevator::ELEVATOR_STATE new_pos) {
    current_state = new_pos;
}

// util methods

/**
 * @brief Resets left and right motor rotation 
 * 
 */
void Elevator::zero_motors() {
    left_.SetSelectedSensorPosition(0);
    right_.SetSelectedSensorPosition(0);
}

/**
 * @brief Stops all motors. Used in situations when estop is triggered,
 * or when elevator height is close to exceeding max distance (22 inches).
 * 
 */
void Elevator::stop() {
    setState(ELEVATOR_STATE::STOPPED);
    left_.SetVoltage(units::volt_t{0});
    right_.SetVoltage(units::volt_t{0});
}

/**
 * @brief Method to allow updates to feedforward constants.
 * 
 * @param ks 
 * @param kv 
 * @param kd 
 * @param ka 
 */
void Elevator::setFeedforwardConstants(double ks, double kv, double kg, double ka) {
    feedforward_.setKs(ks);
    feedforward_.setKv(kv);
    feedforward_.setKg(kg);
    feedforward_.setKa(ka);
}

void Elevator::setPIDConstants(double kp, double kd) {
    feedforward_.setPIDConstants(kp, kd);
}