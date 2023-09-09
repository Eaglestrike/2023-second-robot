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

/**
 * @brief Gets the position of the left motor
 * 
 * @return double the sensor position in raw sensor units
 */
double Elevator::getLeftRotation() {
    return left_.GetSelectedSensorPosition();
}

/**
 * @brief Gets the position of the right motor
 * 
 * @return double the sensor position in raw sensor units
 */
double Elevator::getRightRotation() {
    return right_.GetSelectedSensorPosition();
}

/**
 * @brief Construct a new Elevator:: Elevator object
 */
Elevator::Elevator():
    left_(ElevatorConstants::LEFT_MOTOR_ID, "rio"),
    right_(ElevatorConstants::RIGHT_MOTOR_ID, "rio"),
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
    right_.SetInverted(true);
    right_.Follow(left_, FollowerType::FollowerType_PercentOutput);
    feedforward_.setMaxVelocity(ElevatorConstants::MAX_ELEVATOR_VELOCITY);
    feedforward_.setMaxAcceleration(ElevatorConstants::MAX_ELEVATOR_ACCELERATION);
    feedforward_.setMaxDistance(90); // while testing

    frc::SmartDashboard::PutNumber("max distance", 90.0);
    // feedforward_.setMaxDistance(ElevatorConstants::MAX_ELEVATOR_EXTENSION);
};

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::periodic() {
    setMaxDistance(frc::SmartDashboard::GetNumber("max distance", 90.0));
    if (current_state == STOPPED) {
        return;
    }

    Poses::Pose1D current_values;

    // dividing by 10 to convert from 100 milliseconds to seconds.
    // current_values.velocity = talonUnitsToMeters(left_.GetSelectedSensorVelocity()) * 10.0;
    // current_values.position = talonUnitsToMeters(left_.GetSelectedSensorPosition());

    // used to test elevator on arm rig
    current_values.velocity = talonUnitsToAngle(left_.GetSelectedSensorVelocity()) * 10.0;
    current_values.position = talonUnitsToAngle(left_.GetSelectedSensorPosition());

    double motor_output = feedforward_.periodic(current_values);

    frc::SmartDashboard::PutNumber("current ev velocity", current_values.velocity);
    frc::SmartDashboard::PutNumber("current ev position", current_values.position);

    left_.SetVoltage(units::volt_t{std::clamp(motor_output, -1.0, 1.0)});
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
    feedforward_.reset();
}

/**
 * @brief Stops all motors, and pauses the feedforward calculations
 * 
 */
void Elevator::stop() {
    current_state = ElevatorState::STOPPED;
    left_.SetVoltage(units::volt_t{0});
    right_.SetVoltage(units::volt_t{0});
    feedforward_.stop();
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

/**
 * @brief This function converts the motor units used by the talon to meters.
 * 
 * @param motor_units the raw sensor units
 * @return double the unit in meters
 */
double Elevator::talonUnitsToMeters(double motor_units) {
    return motor_units / ElevatorConstants::TALON_FX_COUNTS_PER_REV * ElevatorConstants::ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED;
}

/**
 * @brief Converts the talon motor units to an angle in degrees
 *  
 * @param motor_units the raw sensor units
 * @return double the angle of the motor in degrees
 */
double Elevator::talonUnitsToAngle(double motor_units) {
    return int(motor_units * 360.0 / ElevatorConstants::TALON_FX_COUNTS_PER_REV) % 360;
}

Elevator::ElevatorState Elevator::getState() {
    return current_state;
}

void Elevator::start() {
    feedforward_.start();
}