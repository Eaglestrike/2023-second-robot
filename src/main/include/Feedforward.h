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

class Feedforward {
    public:
        struct Pose {
            double velocity;
            double acceleration;
            double distance;
        };

        double calculate(double velocity, double acceleration);
        double getExpectedPose(double distance);

        // getters and setters
        double getKs();
        double getKv();
        double getKa();
        double getKg();

        void setKs(double ks);
        void setKv(double kv);
        void setKa(double ka);
        void setKg(double kg);


    private:
        // member functions
        double sign(double value);

        // feedforward constants
        double ks, kv, ka, kg;

        // pid constants
        double kp, ki, kd;
};