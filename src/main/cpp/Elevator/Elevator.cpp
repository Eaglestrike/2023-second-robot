//
// Created by Vir Shah on 6/14/23.
//

#include "Elevator/Elevator.h"
#include "frc/controller/ElevatorFeedforward.h"
#include <stdio.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
    left_(leftID, "rio"),
    right_(rightID, "rio"),
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
    feedforward_.setMaxVelocity(ElevatorConstants::MAX_ELEVATOR_VELOCITY);
    feedforward_.setMaxAcceleration(ElevatorConstants::MAX_ELEVATOR_ACCELERATION);
};

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::periodic() {

    if (current_state == STOPPED) {
        return;
    }

    Poses::Pose1D current_values;
    current_values.velocity = left_.GetSelectedSensorVelocity();
    frc::SmartDashboard::PutNumber("current ev velocity", current_values.velocity);
    current_values.position = (left_.GetSelectedSensorPosition() / ElevatorConstants::TALON_FX_COUNTS_PER_REV) * ElevatorConstants::ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED;
    // current_values.position = left_.GetSelectedSensorPosition();
    frc::SmartDashboard::PutNumber("current ev position", current_values.position);

    double motor_output = feedforward_.periodic(current_values);

    // sketch
    // if (current_state == MOVING_TO_DOCKED) {
    //     motor_output *= -1.0;
    // }
    // motor_output = 0.65;
    frc::SmartDashboard::PutNumber("motor output", motor_output);

    left_.SetVoltage(units::volt_t{std::clamp(motor_output, 0.0, 2.0)});
    right_.SetInverted(true);
    right_.Follow(left_, FollowerType::FollowerType_PercentOutput);
}

/**
 * This will be used to set the next position that the elevator should move to 
 * @param new_pos the next state that the elevator should be in
 */
void Elevator::setState(Elevator::ElevatorState new_pos) {
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
    setState(ElevatorState::STOPPED);
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

void Elevator::setMaxDistance(double distance) {
    feedforward_.setMaxDistance(distance);
}