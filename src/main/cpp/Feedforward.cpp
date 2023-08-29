#include "Feedforward.h"

/**
 * @brief Basic constructor meant for feedforward without PID use.
 *
 * @param ks static constant
 * @param kv velocity to volts constant
 * @param ka acceleration to volts constant
 * @param kg constant to account for acceleration of gravity
 * @param distance the total distance needed to travel by the system
 */
Feedforward::Feedforward(double ks, double kv, double ka, double kg, double distance)
{
    this->ks = ks;
    this->kv = kv;
    this->ka = ka;
    this->kg = kg;
    this->max_distance_ = distance;
}

/**
 * @brief Constructor meant for when you want to initialize PID constants.
 *
 * @param ks static constant
 * @param kv velocity to volts constant
 * @param ka acceleration to volts constant
 * @param kg constant to account for acceleration of gravity
 * @param distance the total distance needed to travel by the system
 * @param kp change in velocity to volts constant
 * @param kd change in position to volts constant
 */
Feedforward::Feedforward(double ks, double kv, double ka, double kg, double kp, double kd, double distance)
{
    this->ks = ks;
    this->kv = kv;
    this->ka = ka;
    this->kg = kg;
    this->max_distance_ = distance;

    // plus PD constants
    this->kp = kp;
    this->kd = kd;
}

/**
 * @brief Runs every periodic cycle.
 *
 * @param current_values a pair containing the velocity and distance (respectively) of the current system.
 * @return a voltage that a motor is expected to use
 */
double Feedforward::periodic(std::pair<double, double> current_values)
{
    if (!isRunning) {
        start();
    }

    Pose expected_values = getExpectedPose(timer.Get().value());

    double feedforward_voltage = calculate(expected_values.velocity, expected_values.acceleration);
    double pid_voltage = pid_calculations({expected_values.velocity, expected_values.distance}, current_values);

    return feedforward_voltage + pid_voltage;
}

/**
 * @brief PID calculations to make feedforward loop more accurate
 *
 * @param expected Pose containing information about where system should be
 * @param current Pose containing information about where system actually is
 * @return double voltage to add to feedforward loop to compensate for inaccuracies in velocity/position.
 */
double Feedforward::pid_calculations(std::pair<double, double> expected, std::pair<double, double> current)
{
    return kp * (expected.first - current.first) + kd * (expected.second - current.second);
}

/**
 * @brief Returns the voltage based on the feedforward formula
 *
 * @param velocity - expected velocity, based on motion profile
 * @param acceleration - expected acceleration, based on motion profile
 *
 * @return the voltage to move
 */
double Feedforward::calculate(double velocity, double acceleration)
{
    return ks * sign(velocity) + kg + kv * velocity + ka * acceleration;
}

/**
 * @brief The sign of a double
 *
 * @param value the value to get the sign of
 * @return 1.0 if the value is greater than 0, or -1.0 otherwise
 */
double Feedforward::sign(double value)
{
    return value > 0.0 ? 1.0 : -1.0;
}

/**
 * @brief Resets and starts the timer
 *
 */
void Feedforward::start()
{
    timer.Reset();
    timer.Start();
    isRunning = true;
}

/**
 * @brief A feedforward function that gets the elevator pose based on current time
 *
 * @param time current system time
 * @return pose a pose containing the distance
 */
Feedforward::Pose Feedforward::getExpectedPose(double time)
{
    Pose pose;

    // the time spent accelerating (or decelerating)
    double acceleration_time = max_velocity / max_acceleration;

    // the time spent maintaining a constant velocity
    double velocity_time = (max_distance_ - max_velocity * acceleration_time) / max_velocity;

    // if in the acceleration phase
    if (0 < time && time < acceleration_time)
    {
        pose.acceleration = max_acceleration;
        pose.velocity = max_acceleration * time;
        pose.distance = 0.5 * pose.velocity * time;
    }

    // if in the velocity phase
    else if (time < acceleration_time + velocity_time)
    {
        pose.acceleration = 0.0;
        pose.velocity = max_velocity;
        // adds phase 1 to however much of phase 2 has been gone through
        pose.distance = 0.5 * max_velocity * acceleration_time + max_velocity * (time - acceleration_time);
    }

    // if in the deceleration phase
    else
    {
        pose.acceleration = -1.0 * max_acceleration;
        pose.velocity = max_velocity - (max_acceleration * (time - (acceleration_time + velocity_time)));
        pose.distance = 0.5 * max_velocity * acceleration_time + max_velocity * velocity_time + (pose.velocity * pose.velocity - max_velocity * max_velocity) / (2.0 * max_acceleration);
    }

    return pose;
}

/**
 * @brief Optional method that must be called if PID calculations are wanted.
 *
 * @param kp kp constant, converts change in velocity to voltage
 * @param kd kd constant, converts change in position to voltage
 */
void Feedforward::setPIDConstants(double kp, double kd)
{
    this->kp = kp;
    this->kd = kd;
}

// getters and setters

double Feedforward::getKs() {
    return ks;
}

void Feedforward::setKs(double ks) {
    this->ks = ks;
}


double Feedforward::getKv() {
    return kv;
}

void Feedforward::setKv(double kv) {
    this->kv = kv;
}


double Feedforward::getKa() {
    return ka;
}

void Feedforward::setKa(double ka) {
    this->ka = ka;
}

double Feedforward::getKp() {
    return kp;
}

double Feedforward::getKd() {
    return kd;
}

double Feedforward::getMaxAcceleration() {
    return max_acceleration;
}

void Feedforward::setMaxAcceleration(double new_acc) {
    max_acceleration = new_acc;
}


double Feedforward::getMaxVelocity() {
    return max_velocity;
}

void Feedforward::setMaxVelocity(double new_vel) {
    max_velocity = new_vel;
}