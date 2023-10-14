#include "Elevator/FeedforwardPID.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * @brief Basic constructor meant for feedforward without PID use.
 *
 * @param ks static constant
 * @param kv velocity to volts constant
 * @param ka acceleration to volts constant
 * @param kg constant to account for acceleration of gravity
 * @param distance the total distance needed to travel by the system
 */
FeedforwardPID::FeedforwardPID(double ks, double kv, double ka, double kg, double distance):
    ks(ks), kv(kv), ka(ka), kg(kg), max_distance_(distance) {
       recalculateTimes(); 
    };

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
FeedforwardPID::FeedforwardPID(double ks, double kv, double ka, double kg, double kp, double ki, double kd, double distance): 
    ks(ks), kv(kv), ka(ka), kg(kg), kp(kp), ki(ki), kd(kd), max_distance_(distance) {
        frc::SmartDashboard::PutNumber("position error", 0.0);
        recalculateTimes();
    };

/**
 * @brief Runs every periodic cycle.
 *
 * @param current_values a pair containing the velocity and distance (respectively) of the current system.
 * @return a voltage that a motor is expected to use
 */
double FeedforwardPID::periodic(Poses::Pose1D current_values)
{
    if (!isRunning) {
        start();
    }

    Poses::Pose1D expected_pose = getExpectedPose(timer.Get().value());
    double feedforward_voltage = calculateFeedforwardVoltage(expected_pose.velocity, expected_pose.acceleration);
    double pid_voltage = calculatePIDVoltage({expected_pose.velocity, expected_pose.position}, current_values);

    // debug prints
    frc::SmartDashboard::PutNumber("timer value: ", timer.Get().value());

    frc::SmartDashboard::PutNumber("expected ev velocity", expected_pose.velocity);
    frc::SmartDashboard::PutNumber("expected ev position", expected_pose.position);
    frc::SmartDashboard::PutNumber("expected acceleration", expected_pose.acceleration);

    frc::SmartDashboard::PutNumber("position error", expected_pose.position - current_values.position);
    frc::SmartDashboard::PutNumber("velocity error", expected_pose.velocity - current_values.velocity);
    frc::SmartDashboard::PutNumber("acceleration error", expected_pose.acceleration - current_values.acceleration);

    frc::SmartDashboard::PutNumber("ff voltage", feedforward_voltage);
    frc::SmartDashboard::PutNumber("pid voltage", pid_voltage);

    return feedforward_voltage + pid_voltage;
}

/**
 * @brief PID calculations to make feedforward loop more accurate
 *
 * @param expected Pose containing information about where system should be
 * @param current Pose containing information about where system actually is
 * @return double voltage to add to feedforward loop to compensate for inaccuracies in velocity/position.
 */
double FeedforwardPID::calculatePIDVoltage(Poses::Pose1D expected, Poses::Pose1D current)
{
    return kp * (expected.position - current.position) + kd * (expected.velocity - current.velocity);
}

/**
 * @brief Returns the voltage based on the feedforward formula
 *
 * @param velocity - expected velocity, based on motion profile
 * @param acceleration - expected acceleration, based on motion profile
 *
 * @return the voltage to move
 */
double FeedforwardPID::calculateFeedforwardVoltage(double velocity, double acceleration)
{
    return ks * sign(velocity) + kg + kv * velocity + ka * acceleration;
}

/**
 * @brief The sign of a double
 *
 * @param value the value to get the sign of
 * @return 1.0 if the value is greater than 0, or -1.0 otherwise
 */
double FeedforwardPID::sign(double value)
{
    return value > 0.0 ? 1.0 : -1.0;
}

/**
 * @brief Starts the timer
 *
 */
void FeedforwardPID::start()
{
    timer.Start();
    isRunning = true;
}

/**
 * @brief Stops the timer
 *  
 */
void FeedforwardPID::stop() {
    timer.Stop();
}

/**
 * Resets and starts the timer 
 * 
 */
void FeedforwardPID::reset() {
    timer.Reset();
    start();
}

/**
 * @brief A feedforward function that gets the elevator pose based on current time
 *
 * @param time current system time
 * @return pose a pose containing the distance
 */
Poses::Pose1D FeedforwardPID::getExpectedPose(double time)
{
    Poses::Pose1D pose;

    // whether moving up or down
    double reversed_coefficient = reversed ? -1.0 : 1.0;

    frc::SmartDashboard::PutBoolean("phase 1", false);
    frc::SmartDashboard::PutBoolean("phase 2", false);
    frc::SmartDashboard::PutBoolean("phase 3", false);
    frc::SmartDashboard::PutBoolean("Reversed?", reversed);

    // if in the acceleration phase
    if (0 < time && time < acceleration_time)
    {
        frc::SmartDashboard::PutBoolean("phase 1", true);
        pose.acceleration = reversed_coefficient * max_acceleration;
        pose.velocity = reversed_coefficient * max_acceleration * time;
        pose.position = reversed_coefficient * 0.5 * pose.velocity * time;
    }

    // if in the velocity phase
    else if (velocity_time != 0 && time < acceleration_time + velocity_time)
    {
        frc::SmartDashboard::PutBoolean("phase 2", true);
        pose.acceleration = 0.0;
        pose.velocity = reversed_coefficient * max_velocity;
        // adds phase 1 to however much of phase 2 has been gone through
        pose.position = reversed_coefficient * 0.5 * max_velocity * acceleration_time + max_velocity * reversed_coefficient * (time - acceleration_time);
    }

    // if in the deceleration phase
    else if (time < velocity_time + acceleration_time * 2)
    {
        frc::SmartDashboard::PutBoolean("phase 3", true);

        double first_phase_distance = reversed_coefficient * 0.5 * max_velocity * acceleration_time;
        double second_phase_distance = reversed_coefficient * max_velocity * velocity_time;
        double time_in_triangle = time - (acceleration_time + velocity_time);

        pose.acceleration = -1.0 * reversed_coefficient * max_acceleration;
        pose.velocity = reversed_coefficient * max_velocity - (reversed_coefficient * max_acceleration * (time_in_triangle));

        // trapezoidal area
        double third_phase_distance = reversed_coefficient * (max_velocity + pose.velocity) / 2.0 * time_in_triangle;

        pose.position = first_phase_distance + second_phase_distance + third_phase_distance;
    }

    return pose;
}

/**
 * @brief Optional method that must be called if PID calculations are wanted.
 *
 * @param kp kp constant, converts change in velocity to voltage
 * @param kd kd constant, converts change in position to voltage
 */
void FeedforwardPID::setPIDConstants(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

/**
 * @brief Calculates and stores the time spent in the different phases for the feedforward loop
 * 
 */
void FeedforwardPID::recalculateTimes() {
    // the time spent accelerating (or decelerating)
    if (max_velocity == 0 || max_acceleration == 0) {
        frc::SmartDashboard::PutString("ELEVATOR ERROR", "Either max velocity or max acceleration are 0. Defaulting to 10 for acceleration time and 10 for velocity time.");
        acceleration_time = 10.0;
        velocity_time = 10.0;
    } else {
        acceleration_time = max_velocity / max_acceleration;

        // the time spent maintaining a constant velocity
        velocity_time = (max_distance_ - max_velocity * acceleration_time) / max_velocity;

        if (velocity_time < 0) {
            velocity_time = 0.0;
            acceleration_time = std::sqrt(max_distance_ / max_acceleration);
        }
    }

}

// getters and setters

double FeedforwardPID::getKs() {
    return ks;
}

void FeedforwardPID::setKs(double ks) {
    this->ks = ks;
}

void FeedforwardPID::setKg(double kg) {
    this->kg = kg;
}

double FeedforwardPID::getKv() {
    return kv;
}

void FeedforwardPID::setKv(double kv) {
    this->kv = kv;
}


double FeedforwardPID::getKa() {
    return ka;
}

void FeedforwardPID::setKa(double ka) {
    this->ka = ka;
}

double FeedforwardPID::getKp() {
    return kp;
}

double FeedforwardPID::getKd() {
    return kd;
}

double FeedforwardPID::getMaxAcceleration() {
    return max_acceleration;
}

void FeedforwardPID::setMaxAcceleration(double new_acc) {
    max_acceleration = new_acc;
    recalculateTimes();
}

double FeedforwardPID::getMaxVelocity() {
    return max_velocity;
}

void FeedforwardPID::setMaxVelocity(double new_vel) {
    max_velocity = new_vel;
    recalculateTimes();
}

void FeedforwardPID::setMaxDistance(double distance) {
    max_distance_ = distance;
    recalculateTimes();
}

bool FeedforwardPID::getReversed() {
    return reversed;
}

void FeedforwardPID::setReversed(bool reversed) {
    this->reversed = reversed;
}