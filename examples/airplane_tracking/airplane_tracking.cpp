#include "unscented/primitives.hpp"
#include "unscented/ukf.hpp"

#include "airplane_state.h"
#include "radar_measurement.h"

#include <iostream>
#include <random>

int main()
{
  //////////////////////////////////////////////////////////////////////////////
  // Setup the filter
  //////////////////////////////////////////////////////////////////////////////

  // The airplane state has four degrees of freedom (position, velocity,
  // altitude, climb rate) and the radar measurement has two degrees of freedom
  // (range, elevation)
  using UKF = unscented::UKF<AirplaneState, RadarMeasurement>;
  UKF ukf(unscented::mean_function<AirplaneState, UKF::NUM_SIGMA_POINTS>,
          mean_function<UKF::NUM_SIGMA_POINTS>);

  // Simulation parameters
  const auto SIM_DURATION = 360.0; // seconds
  const auto DT = 3.0; // seconds

  // Calculate process noise covariance Q using the discrete constant white
  // noise model (Bar-Shalom. “Estimation with Applications To Tracking and
  // Navigation”. John Wiley & Sons, 2001. Page 274.). To simplify, one can
  // simply pick (or tune) appropriate values on the diagonal of Q.
  const auto PROCESS_VAR = 0.1;
  Eigen::Vector2d G(0.5 * DT * DT, DT);
  Eigen::Matrix4d Q;
  Q.block<2, 2>(0, 0) = G * G.transpose() * PROCESS_VAR;
  Q.block<2, 2>(2, 2) = G * G.transpose() * PROCESS_VAR;
  ukf.process_covariance(Q);

  // Calculate measurement noise covariance R (standard deviations chosen
  // somewhat arbitrarily)
  const auto RANGE_STD_DEV = 5.0; // meters
  const auto ELEVATION_STD_DEV = 0.5 * M_PI / 180.0; // radians
  UKF::M_by_M R;
  R << std::pow(RANGE_STD_DEV, 2), 0.0, 0.0, std::pow(ELEVATION_STD_DEV, 2);
  ukf.measurement_covariance(R);

  // Set initial state estimate and its covariance
  AirplaneState true_state(0, 100, 1000, 0);
  AirplaneState initial_state_estimate(0, 90, 1100, 0);
  ukf.state(initial_state_estimate);
  UKF::N_by_N P = UKF::N_by_N::Zero();
  P(0, 0) = std::pow(300.0, 2); // m^2
  P(1, 1) = std::pow(30.0, 2); // (m/s)^2
  P(2, 2) = std::pow(150.0, 2); // m^2
  P(3, 3) = std::pow(3.0, 2); // (m/s)^2
  ukf.state_covariance(P);

  // Create vectors that will hold the histories of the true and estimated
  // states, populating them both with the initial states. This is simply
  // recorded so the results over time can be plotted at the end of the
  // simulation.
  std::vector<AirplaneState> true_state_history;
  true_state_history.push_back(true_state);
  std::vector<AirplaneState> estimated_state_history;
  estimated_state_history.push_back(ukf.state());
  std::vector<UKF::N_by_N> estimated_state_cov_history;

  // Setup random number generation
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> vel_noise(0.0, 0.02);
  std::normal_distribution<double> range_noise(0.0, RANGE_STD_DEV);
  std::normal_distribution<double> elevation_noise(0.0, ELEVATION_STD_DEV);

  //////////////////////////////////////////////////////////////////////////////
  // Run the simulation
  //////////////////////////////////////////////////////////////////////////////

  double sim_time = 0.0;
  std::vector<double> sim_time_history;
  while (sim_time <= SIM_DURATION)
  {
    // One minute into the simulation, set a non-zero climb rate
    if (sim_time > 60.0)
    {
      true_state[CLIMB_RATE] = 300.0 / 60; // 300 m/min
    }

    // Update the true position and altitude, adding some noise to the velocity
    // to make up for deficiencies in the constant velocity model (i.e., it's
    // unlikely the velocity was constant throughout the full time step)
    true_state[POSITION] += (true_state[VELOCITY] + vel_noise(gen)) * DT;
    true_state[ALTITUDE] += (true_state[CLIMB_RATE] + vel_noise(gen)) * DT;
    true_state_history.push_back(true_state);

    // Simulate a measurement based on the true state
    auto meas = measurement_model(true_state);
    meas.range += range_noise(gen);
    meas.elevation =
        unscented::Rotation2d(meas.elevation.angle() + elevation_noise(gen));

    // Update the filter estimates
    ukf.predict(system_model, DT);
    ukf.correct(measurement_model, meas);

    // Record all the current values in the history
    estimated_state_history.push_back(ukf.state());
    estimated_state_cov_history.push_back(ukf.state_covariance());
    sim_time_history.push_back(sim_time);

    // Move time forward
    sim_time += DT;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Output the results
  //////////////////////////////////////////////////////////////////////////////

  std::cout << "time,true_pos,est_pos,est_pos_std_dev,true_vel,est_vel,est_vel_"
               "std_dev,true_alt,est_alt,est_alt_std_dev,true_climb,est_climb,"
               "est_climb_std_dev\n";
  for (auto i = 0; i < sim_time_history.size(); ++i)
  {
    const auto& timestamp = sim_time_history[i];
    const auto& true_state = true_state_history[i];
    const auto& est_state = estimated_state_history[i];
    const auto& est_state_cov = estimated_state_cov_history[i];
    std::cout << timestamp << "," << true_state[POSITION] << ","
              << est_state[POSITION] << ","
              << std::sqrt(est_state_cov(POSITION, POSITION)) << ","
              << true_state[VELOCITY] << "," << est_state[VELOCITY] << ","
              << std::sqrt(est_state_cov(VELOCITY, VELOCITY)) << ","
              << true_state[ALTITUDE] << "," << est_state[ALTITUDE] << ","
              << std::sqrt(est_state_cov(ALTITUDE, ALTITUDE)) << ","
              << true_state[CLIMB_RATE] << "," << est_state[CLIMB_RATE] << ","
              << std::sqrt(est_state_cov(CLIMB_RATE, CLIMB_RATE)) << "\n";
  }

  return 0;
}
