#include "Feedforward.h"

/**
 * @brief Returns the voltage based on the feedforward formula
 *  
 * @param velocity - expected velocity, based on motion profile
 * @param acceleration - expected acceleration, based on motion profile
 * 
 * @return the voltage to move
 */
double Feedforward::calculate(double velocity, double acceleration) {
    return ks * sign(velocity) + kg + kv * velocity + ka * acceleration;
}

/**
 * @brief The sign of a double
 * 
 * @param value the value to get the sign of
 * @return 1.0 if the value is greater than 0, or -1.0 otherwise 
 */
double Feedforward::sign(double value) {
    return value > 0.0 ? 1.0 : -1.0;
}

/**
 * @brief Gets the elevator pose
 * 
 * @param distance 
 * @return double 
 */
double Feedforward::getExpectedPose(double distance) {

}