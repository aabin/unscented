#ifndef UNSCENTED_AIRPLANE_TRACKING_RADAR_MEAS_H
#define UNSCENTED_AIRPLANE_TRACKING_RADAR_MEAS_H

#include "unscented/primitives.hpp"
#include "unscented/ukf.hpp"

#include "airplane_state.h"

/**
 * @brief The radar measurement consists of a range and elevation
 */
struct RadarMeasurement
{
  /** The number of degrees of freedom of this measurement (required by the
   * filter) */
  static constexpr std::size_t DOF = 2;

  /**
   * @brief Default constructor (required by filter)
   */
  RadarMeasurement() = default;

  /**
   * @brief Construct taking a vector with dimension the same as the DOF
   * (required by filter)
   *
   * @param[in] vec Vector containing range and elevation of the radar
   * measurement
   */
  explicit RadarMeasurement(const unscented::Vector<2>& vec)
    : range(vec(0)), elevation(vec(1))
  {
  }

  /**
   * @brief Constructor populating range and elevation (not required
   * by filter, included for convenience)
   *
   * @param[in] r Measured range to airplane
   * @param[in] e Measured elevation of airplane
   */
  RadarMeasurement(double r, const unscented::UnitComplex& e)
    : range(r), elevation(e)
  {
  }

  /**
   * @brief Constructor populating range and elevation as a double (not required
   * by filter, included for convenience)
   *
   * @param[in] r Measured range to airplane
   * @param[in] e Measured elevation of airplane
   */
  RadarMeasurement(double r, double e) : range(r), elevation(e)
  {
  }

  /** Range to airplane */
  double range{0.0};

  /** Elevation of airplane */
  unscented::UnitComplex elevation{0.0};
};

/**
 * @brief Adds two measurements together and produces another measurement
 * (required by the filter).
 *
 * @param[in] lhs
 * @param[in] rhs
 *
 * @return Sum of two measurements, automatically handles wrapping of elevation
 * due to use of a unit complex number to represent it
 */
RadarMeasurement operator+(const RadarMeasurement& lhs,
                           const RadarMeasurement& rhs)
{
  return RadarMeasurement(lhs.range + rhs.range, lhs.elevation + rhs.elevation);
}

/**
 * @brief Substracts the rhs measurement from the lhs measurement, with the
 * result put into a vector of size equal to the DOF of the measurement
 * (required by the filter)
 *
 * @param[in] lhs
 * @param[in] rhs
 *
 * @return Difference of two measurements in a vector, automatically handles
 * wrapping of elevation angle due to use of a unit complex number to represent
 * it
 */
unscented::Vector<2> operator-(const RadarMeasurement& lhs,
                               const RadarMeasurement& rhs)
{
  return unscented::Vector<2>(lhs.range - rhs.range,
                              (lhs.elevation - rhs.elevation)(0));
}

template <std::size_t NUM_SIGMA_POINTS>
RadarMeasurement radar_measurement_mean_function(
    const std::array<RadarMeasurement, NUM_SIGMA_POINTS>& states,
    const std::array<double, NUM_SIGMA_POINTS>& weights)
{
  double range{0.0};
  double a{0.0};
  double b{0.0};
  for (std::size_t i = 0; i < NUM_SIGMA_POINTS; ++i)
  {
    range += weights[i] * states[i].range;
    a += weights[i] * states[i].elevation.a;
    b += weights[i] * states[i].elevation.b;
  }
  return RadarMeasurement(range, unscented::UnitComplex(a, b));
}

/**
 * @brief The measurement model maps a state to an expected measurement. In this
 * case, we can use trigonometry to map the position and altitude of the
 * airplane to its range and elevation. This is essentially a Cartesian-to-polar
 * conversion.
 *
 * @param[in] state The state to map to an expected measurement
 *
 * @return The expected measurement
 */
RadarMeasurement measurement_model(const AirplaneState& state)
{
  const auto range =
      std::sqrt(std::pow(state[POSITION], 2) + std::pow(state[ALTITUDE], 2));
  const auto elevation = std::atan2(state[ALTITUDE], state[POSITION]);
  return RadarMeasurement(range, elevation);
}

#endif
