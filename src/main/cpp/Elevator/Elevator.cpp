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
    left_(ElevatorConstants::LEFT_MOTOR_ID, "rio"), right_(ElevatorConstants::RIGHT_MOTOR_ID, "rio"),
    current_state_(STOPPED),
    current_target_(STOWED),
    feedforward_(ElevatorConstants::FEEDFORWARD_CONSTANTS, true),
    max_volts_(ElevatorConstants::MAX_VOLTS)
{
    current_pose_ = {0.0, 0.0, 0.0};

    left_.SetInverted(true);
    right_.Follow(left_, FollowerType::FollowerType_PercentOutput);
};

void Elevator::CorePeriodic(){
    current_pose_.position = talonUnitsToMeters(left_.GetSelectedSensorPosition());
    // dividing by 10 to convert from 100 milliseconds to seconds.
    current_pose_.velocity = talonUnitsToMeters(left_.GetSelectedSensorVelocity()) * 10.0;
}

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::CoreTeleopPeriodic() {
    double motor_output;

    switch(current_state_){
        case MANUAL:
            motor_output = debug_manual_volts_;
            break;
        case HOLDING_POSITION: 
        case MOVING:
            motor_output = feedforward_.periodic(current_pose_);
            break;
        default:
            motor_output = 0.0;
    }

    left_.SetVoltage(units::volt_t{std::clamp(motor_output, -max_volts_, max_volts_)});
    right_.SetVoltage(units::volt_t{std::clamp(motor_output, -max_volts_, max_volts_)});
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

/**
 * @brief Returns the elevator height in meters, calculated per the left motor's position.
 * 
 * @return double height (in meters)
 */
double Elevator::getElevatorHeight() {
    return talonUnitsToMeters(left_.GetSelectedSensorPosition());
}

void Elevator::CoreShuffleboardInit(){
    frc::SmartDashboard::PutNumber(name_ + " ks", ElevatorConstants::KS);
    frc::SmartDashboard::PutNumber(name_ + " kv", ElevatorConstants::KV);
    frc::SmartDashboard::PutNumber(name_ + " ka", ElevatorConstants::KA);
    frc::SmartDashboard::PutNumber(name_ + " kg", ElevatorConstants::KG);

    frc::SmartDashboard::PutNumber(name_ + " kp", ElevatorConstants::KP);
    frc::SmartDashboard::PutNumber(name_ + " kd", ElevatorConstants::KD);

    frc::SmartDashboard::PutNumber(name_ + " mv", ElevatorConstants::MAX_VELOCITY);
    frc::SmartDashboard::PutNumber(name_ + " ma", ElevatorConstants::MAX_ACCELERATION);

    frc::SmartDashboard::PutNumber(name_ + " setPoint", ElevatorConstants::MAX_EXTENSION);

    frc::SmartDashboard::PutNumber(name_ + "volts to use", 0.0);
};

void Elevator::CoreShuffleboardPeriodic(){
    // frc::SmartDashboard::PutNumber(name_ + " lm rotation", getLeftRotation());
    // frc::SmartDashboard::PutNumber(name_ + " rm rotation", getRightRotation());

    frc::SmartDashboard::PutNumber("current ev position", current_pose_.position);
    frc::SmartDashboard::PutNumber("current ev velocity", current_pose_.velocity);
};

void Elevator::CoreShuffleboardUpdate(){
    feedforward_.setKs(frc::SmartDashboard::GetNumber(name_ + " ks", ElevatorConstants::KS));
    feedforward_.setKv(frc::SmartDashboard::GetNumber(name_ + " kv", ElevatorConstants::KV));
    feedforward_.setKa(frc::SmartDashboard::GetNumber(name_ + " ka", ElevatorConstants::KA));
    feedforward_.setKg(frc::SmartDashboard::GetNumber(name_ + " kg", ElevatorConstants::KG));

    feedforward_.setPIDConstants(frc::SmartDashboard::GetNumber(name_ + " kp", ElevatorConstants::KP),
                                 frc::SmartDashboard::GetNumber(name_ + " kd", ElevatorConstants::KD));

    feedforward_.setMaxVelocity(frc::SmartDashboard::GetNumber(name_ + " mv", ElevatorConstants::MAX_VELOCITY));
    feedforward_.setMaxAcceleration(frc::SmartDashboard::GetNumber(name_ + " ma", ElevatorConstants::MAX_ACCELERATION));

    double setPoint = frc::SmartDashboard::GetNumber(name_ + " setPoint", ElevatorConstants::MAX_EXTENSION);
    if (setPoint < ElevatorConstants::MAX_EXTENSION) {
        feedforward_.setTotalDistance(setPoint, getElevatorHeight());
    }

    max_volts_ = frc::SmartDashboard::GetNumber("volts to use", 0.0);
};

/**
 * @brief Given a position, the elevator will move to that position
 * 
 * @param newPos 
 */
void Elevator::ExtendToCustomPos(double newPos) {
    current_state_ = ElevatorState::MANUAL;
    feedforward_.setTotalDistance(newPos, getElevatorHeight());
}

void Elevator::ExtendLow() {
    current_target_ = ElevatorTarget::LOW;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT[current_target_], getElevatorHeight());
}

void Elevator::ExtendMid() {
    current_target_ = ElevatorTarget::MID;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT[current_target_], getElevatorHeight());
}

void Elevator::ExtendHigh() {
    current_target_ = ElevatorTarget::HIGH;
    feedforward_.setTotalDistance(ElevatorConstants::TARGET_TO_HEIGHT[current_target_], getElevatorHeight());
}

/**
 * @brief Runs a fraction of the max voltage to the elevator
 * 
 * @param range the range of the xbox joystick axis
 * Note: it is assumed that the range will be from -1 to 1.
 */
void Elevator::setDebugManualVolts(double range) {
    debug_manual_volts_ = range * max_volts_;
}

void Elevator::activateManualMode() {
    current_state_ = ElevatorState::MANUAL;
}

void Elevator::activateMovingMode() {
    current_state_ = ElevatorState::MOVING;
}