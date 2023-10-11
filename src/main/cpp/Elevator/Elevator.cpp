//
// Created by Vir Shah on 6/14/23.
//

#include "Elevator/Elevator.h"

#include <stdio.h>
#include <iostream>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
/**
 * @brief Construct a new Elevator:: Elevator object
 */
Elevator::Elevator(bool enabled, bool shuffleboard):
    Mechanism("elevator", enabled, shuffleboard),
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
    // right_.SetInverted(true);
    left_.SetInverted(true);
    right_.Follow(left_, FollowerType::FollowerType_PercentOutput);
    feedforward_.setMaxVelocity(ElevatorConstants::MAX_ELEVATOR_VELOCITY);
    feedforward_.setMaxAcceleration(ElevatorConstants::MAX_ELEVATOR_ACCELERATION);
    feedforward_.setMaxDistance(ElevatorConstants::MAX_ELEVATOR_EXTENSION); // while testing
};

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::CoreTeleopPeriodic() {
    Poses::Pose1D current_pose;
    current_pose.position = talonUnitsToMeters(left_.GetSelectedSensorPosition());
    // dividing by 10 to convert from 100 milliseconds to seconds.
    current_pose.velocity = talonUnitsToMeters(left_.GetSelectedSensorVelocity()) * 10.0;

    evaluateState();

    double motor_output = feedforward_.periodic(current_pose);

    frc::SmartDashboard::PutNumber("current ev velocity", current_pose.velocity);
    frc::SmartDashboard::PutNumber("current ev position", current_pose.position);

    double volts_to_use = frc::SmartDashboard::GetNumber("volts to use", 0.0);

    left_.SetVoltage(units::volt_t{std::clamp(motor_output, -volts_to_use, volts_to_use)});
    right_.SetVoltage(units::volt_t{std::clamp(motor_output, -volts_to_use, volts_to_use)});
}

/**
 * @brief Takes in current values and decides whether to move from the "MOVING_TO" state to the location state.
 */
void Elevator::evaluateState() {
    //double left_position = talonUnitsToMeters(left_.GetSelectedSensorPosition());

    // if (std::abs(ElevatorConstants::MAX_ELEVATOR_EXTENSION - left_position) < ElevatorConstants::POSITION_ERROR_TOLERANCE) {
    //     if (current_state_ == MOVING_TO_DOCKED) {
    //         current_state_ = DOCKED;
    //     }

    //     if (current_state_ == MOVING_TO_RAISED) {
    //         current_state_ = RAISED;
    //     }
    // }
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

// debug getters
Elevator::ElevatorState Elevator::getState() {
    return current_state_;
}

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

void Elevator::CoreShuffleboardInit(){
    frc::SmartDashboard::PutNumber(name_ + " ks", ElevatorConstants::KS);
    frc::SmartDashboard::PutNumber(name_ + " kv", ElevatorConstants::KV);
    frc::SmartDashboard::PutNumber(name_ + " ka", ElevatorConstants::KA);
    frc::SmartDashboard::PutNumber(name_ + " kg", ElevatorConstants::KG);

    frc::SmartDashboard::PutNumber(name_ + " kp", ElevatorConstants::KP);
    frc::SmartDashboard::PutNumber(name_ + " kd", ElevatorConstants::KD);

    frc::SmartDashboard::PutNumber(name_ + " mv", ElevatorConstants::MAX_ELEVATOR_VELOCITY);
    frc::SmartDashboard::PutNumber(name_ + " ma", ElevatorConstants::MAX_ELEVATOR_ACCELERATION);

    frc::SmartDashboard::PutNumber(name_ + " setPoint", ElevatorConstants::MAX_ELEVATOR_EXTENSION);

    frc::SmartDashboard::PutNumber("volts to use", 0.0);
};

void Elevator::CoreShuffleboardPeriodic(){
    // frc::SmartDashboard::PutNumber(name_ + " lm rotation", getLeftRotation());
    // frc::SmartDashboard::PutNumber(name_ + " rm rotation", getRightRotation());
};

void Elevator::CoreShuffleboardUpdate(){
    feedforward_.setKs(frc::SmartDashboard::GetNumber(name_ + " ks", ElevatorConstants::KS));
    feedforward_.setKv(frc::SmartDashboard::GetNumber(name_ + " kv", ElevatorConstants::KV));
    feedforward_.setKa(frc::SmartDashboard::GetNumber(name_ + " ka", ElevatorConstants::KA));
    feedforward_.setKg(frc::SmartDashboard::GetNumber(name_ + " kg", ElevatorConstants::KG));

    feedforward_.setPIDConstants(frc::SmartDashboard::GetNumber(name_ + " kp", ElevatorConstants::KP),
                                 frc::SmartDashboard::GetNumber(name_ + " kd", ElevatorConstants::KD));

    feedforward_.setMaxVelocity(frc::SmartDashboard::GetNumber(name_ + " mv", ElevatorConstants::MAX_ELEVATOR_VELOCITY));
    feedforward_.setMaxAcceleration(frc::SmartDashboard::GetNumber(name_ + " ma", ElevatorConstants::MAX_ELEVATOR_ACCELERATION));

    double setPoint = frc::SmartDashboard::GetNumber(name_ + " setPoint", ElevatorConstants::MAX_ELEVATOR_EXTENSION);
    if (setPoint < ElevatorConstants::MAX_ELEVATOR_EXTENSION) {
        feedforward_.setMaxDistance(setPoint);
    }
};
