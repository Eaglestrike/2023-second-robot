/**
 * A file of constants
 *
 * @note General constants are not namespaced
 */

#pragma once

#include <cstddef>

const double NEAR_ZERO_TOLERANCE = 0.000001;

namespace ElevatorConstants {
  const double TALON_FX_COUNTS_PER_REV = 2048;

  const double MOTOR_VOLTAGE = 1.0;

  // m/s
  // const double MAX_ELEVATOR_VELOCITY = 4.20116;
  const double MAX_ELEVATOR_VELOCITY = 1.34;
    // const double MAX_ELEVATOR_VELOCITY = 0.2;

  const double MAX_ELEVATOR_ACCELERATION = 0.94;

  // m
  const double MAX_ELEVATOR_EXTENSION = 0.5588;
  // const double MAX_ELEVATOR_EXTENSION = 90;


  const double ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED = 0.0465582;

  // feedforward constants
  const double KS = 0.0;
  const double KV = 0.89;
  const double KA = 0.45;
  const double KG = 2.17;

  // pid constants
  const double KP = 0.5;
  const double KD = 0.6;

  // motor ids
  const int LEFT_MOTOR_ID = 22;
  const int RIGHT_MOTOR_ID = 40;

  // m
  const double POSITION_ERROR_TOLERANCE = 0.1;
}

namespace Poses {
  struct Pose1D {
    double velocity = 0.0;
    double acceleration = 0.0;
    double position = 0.0;
  };
};