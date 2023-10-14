/**
 * A file of constants
 *
 * @note General constants are not namespaced
 */

#pragma once

#include <cstddef>

namespace Poses {
    struct Pose1D {
        double velocity = 0.0;
        double acceleration = 0.0;
        double position = 0.0;
    };
};

namespace ElevatorConstants {
  const double TALON_FX_COUNTS_PER_REV = 2048;
  const double ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED = 0.0465582;

  // motor ids
  const int LEFT_MOTOR_ID = 22;
  const int RIGHT_MOTOR_ID = 40;

  const double MAX_VOLTS = 1.0;

  // const double MAX_ELEVATOR_VELOCITY = 4.20116;
  const double MAX_VELOCITY = 1.34; //m/s
  const double MAX_ACCELERATION = 0.94; //m/s^2

  const double MAX_EXTENSION = 0.5588; // m

  //Feedforward Constants
  const double KS = 0.0;
  const double KV = 0.89;
  const double KA = 0.45;
  const double KG = 2.17;

  const double KP = 3.5;
  const double KD = 0.5;

  const struct FeedforwardConfig{
    double ks = KS;
    double kv = KV;
    double ka = KA;
    double kg = KG;
    double maxVel = MAX_VELOCITY;
    double maxAccel = MAX_ACCELERATION;
    double kp = KP;
    double kd = KD;
  } FEEDFORWARD_CONSTANTS;

  const double POSITION_ERROR_TOLERANCE = 0.1; //m
};