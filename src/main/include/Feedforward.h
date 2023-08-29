/**
 * @file Feedforward.h
 * @author Vir Shah (vir.shah@team114.org)
 * @brief Module for feedforward functions, coupled with PID calculator.
 * @version 0.1
 * @date 2023-08-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <frc/Timer.h>
#include <units/velocity.h>
#include <units/acceleration.h>

class Feedforward {
    public:
        struct Pose {
            double velocity;
            double acceleration;
            double distance;
        };

        double calculate(double velocity, double acceleration);
        Pose getExpectedPose(double distance);
        void start();
        double pid_calculations(Pose expected, Pose current);
        double periodic(Pose current_values);

        // getters and setters
        double getKs();
        double getKv();
        double getKa();
        double getKg();

        double getMaxVelocity();
        double getMaxAcceleration();

        void setKs(double ks);
        void setKv(double kv);
        void setKa(double ka);
        void setKg(double kg);

        void setPIDConstants(double kp, double kd);

        void setMaxVelocity(double max_vel);
        void setMaxAcceleration(double max_acc);

        // constructor
        Feedforward(double ks, double kv, double ka, double kg, double distance);
        Feedforward(double ks, double kv, double ka, double kg, double kp, double kd, double distance);

    private:
        // member functions
        double sign(double value);

        // total distance needed to travel
        double max_distance_;

        // feedforward constants
        double ks, kv, ka, kg;

        // pid constants, initialized to 0 so their use is optional
        double kp = 0.0;
        double kd = 0.0;

        // velocity, acceleration constants
        double max_velocity;
        double max_acceleration;

        // timer
        frc::Timer timer{};
};