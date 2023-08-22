//
// Created by Vir Shah on 6/14/23.
//

#include "Elevator.h"
#include "frc/controller/ElevatorFeedforward.h"

// debug getters
double Elevator::getElevatorHeight() {
    return elevator_height;
}

double Elevator::getLeftRotation() {
    return left_motor_rotation;
}

double Elevator::getRightRotation() {
    return right_motor_rotation;
}

/**
 * @brief Construct a new Elevator:: Elevator object
 * 
 * @param ks ks constant -- static friction
 * @param kv kv constant -- velocity to volts coefficient
 * @param ka ka constant -- acceleration to volts coefficient
 * @param kg kg constant -- gravity coefficient
 * @param leftID the ID for the left motor
 * @param rightID  the ID for the right motor
 */
Elevator::Elevator(double ks, double kv, double ka, double kg, int leftID, int rightID): ks(ks), kv(kv), ka(ka), kg(kg), left(leftID, "canbus"), right(rightID, "canbus") {};

/**
 * Called every periodic cycle, handles all movement
 * Acts on the difference between current position and next position
 */
void Elevator::periodic() {

    if (current_state == STOPPED) {
        return;
    }

    if (current_state == DOCKED) {
        // take action to ensure it is still docked
    } else if (current_state == RAISED) {
        // take action to ensure still raised
    } else if (current_state == MOVING_TO_DOCKED) {
        // continue docking process
    } else if (current_state == MOVING_TO_RAISED) {
        // continue raising process
    }

    left_.SetVoltage(ElevatorConstants::MOTOR_VOLTAGE_t);
    right_.SetVoltage(-1.0*ElevatorConstants::MOTOR_VOLTAGE_t);
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
 * Calculates the value based on the feedforward loop
 * @param velocity - expected velocity, based on motion profile
 * @param acc - expected acceleration, based on motion profile
 * @return the values that the motor should move to get to the desired state
 */
double Elevator::calculateFeedforward(double velocity, double acc) {
    return kS * wpi::sgn(velocity) + kG + kV * velocity + kA * acceleration;
}


/**
 * @brief Resets elevator height, left and right motor rotation 
 * 
 */
void Elevator::zero() {
    left_motor_rotation = 0.0;
    right_motor_rotation = 0.0;
    elevator_height = 0.0;
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
 * @brief Uses distance travelled in the motion profile to determine what velocity and acceleration should be.
 * 
 * @return Elevator::ElevatorPose a struct combining velocity, acceleration, and distance
 */
Elevator::ElevatorPose Elevator::getExpectedPose() {
    // elevator_height -> current elevator height
    
}