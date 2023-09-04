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
  const double MAX_ELEVATOR_VELOCITY = 4.20116;
  const double MAX_ELEVATOR_ACCELERATION = 0.1;

  // m
  const double MAX_ELEVATOR_EXTENSION = 0.5588;

  const double ONE_MOTOR_REVOLUTION_TO_DISTANCE_TRAVELLED = 0.0465582;

  // feedforward constants
  const double KS = 0.65;
  const double KV = 0.2;
  const double KA = 0.2;
  const double KG = 0.2;

  // pid constants
  const double KP = 0.0;
  const double KD = 0.0;
}

namespace Poses {
  struct Pose1D {
    double velocity;
    double acceleration;
    double position;
  };
};