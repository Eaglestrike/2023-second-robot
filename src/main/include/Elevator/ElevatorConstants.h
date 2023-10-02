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
    const double MAX_ELEVATOR_VELOCITY = 0.2;

  const double MAX_ELEVATOR_ACCELERATION = 0.8;

  // m
  // const double MAX_ELEVATOR_EXTENSION = 0.5588;
  const double MAX_ELEVATOR_EXTENSION = 90;

  const double ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED = 0.0465582;

  // feedforward constants
  const double KS = 0.62;
  const double KV = 3.0917; // found through spreadsheet analysis
  const double KA = 0.0;
  const double KG = 0.0;

  // pid constants
  const double KP = 0.0;
  const double KD = 0.0;

  // motor ids
  const int LEFT_MOTOR_ID = 2;
  const int RIGHT_MOTOR_ID = 0;

  // m
  const double POSITION_ERROR_TOLERANCE = 0.1;
}

namespace Poses {
  struct Pose1D {
    double velocity;
    double acceleration;
    double position;
  };
};