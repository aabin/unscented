#ifndef UNSCENTED_UKF_HPP
#define UNSCENTED_UKF_HPP

#include "unscented/ukf.h"

#include <cassert>
#include <numeric>

namespace unscented
{
template <typename T, std::size_t SIZE>
T mean_function(const std::array<T, SIZE>& values,
                const std::array<double, SIZE>& weights)
{
  T mean_value;
  for (std::size_t i = 0; i < SIZE; ++i)
  {
    mean_value = mean_value + weights[i] * values[i];
  }
  return mean_value;
}

// Need to explicitly provide the definitions of N, M, and NUM_SIGMA_POINTS
// despite their declaration and initialization being in the .h file (see
// https://stackoverflow.com/q/8016780 for more info)
template <typename STATE, typename MEAS>
constexpr std::size_t UKF<STATE, MEAS>::N;

template <typename STATE, typename MEAS>
constexpr std::size_t UKF<STATE, MEAS>::M;

template <typename STATE, typename MEAS>
constexpr std::size_t UKF<STATE, MEAS>::NUM_SIGMA_POINTS;

template <typename STATE, typename MEAS>
UKF<STATE, MEAS>::UKF(StateMeanFunction state_mean_function,
                      MeasurementMeanFunction meas_mean_function)
  : P_(N_by_N::Identity())
  , Q_(N_by_N::Identity())
  , R_(M_by_M::Identity())
  , Pyy_(M_by_M::Identity())
  , state_mean_function_(std::move(state_mean_function))
  , meas_mean_function_(std::move(meas_mean_function))
{
  calculate_weights();
}

template <typename STATE, typename MEAS>
template <typename SYS_MODEL, typename... PARAMS>
void UKF<STATE, MEAS>::predict(const SYS_MODEL& system_model, PARAMS... params)
{
  generate_sigma_points();

  // Transform each sigma point through the system model
  for (auto& sigma_point : sigma_points_)
  {
    system_model(sigma_point, params...);
  }

  // The (a priori) state is the weighted mean of the transformed sigma points
  x_ = state_mean_function_(sigma_points_, sigma_weights_mean_);

  // Calculate the state covariance
  P_ = Q_;
  for (std::size_t i = 0; i < NUM_SIGMA_POINTS; ++i)
  {
    const N_by_1 diff = sigma_points_[i] - x_;
    P_ += sigma_weights_cov_[i] * (diff * diff.transpose());
  }
}

template <typename STATE, typename MEAS>
template <typename MEAS_MODEL, typename... PARAMS>
void UKF<STATE, MEAS>::correct(const MEAS_MODEL& meas_model, PARAMS... params)
{
  generate_sigma_points();

  // Transform each sigma point through the measurement model
  for (std::size_t i = 0; i < NUM_SIGMA_POINTS; ++i)
  {
    meas_sigma_points_[i] = meas_model(sigma_points_[i], params...);
  }

  // The expected measurement is the weighted mean of the measurement sigma
  // points
  y_hat_ = meas_mean_function_(meas_sigma_points_, sigma_weights_mean_);

  // Calculate the expected measurement covariance
  Pyy_ = R_;
  for (std::size_t i = 0; i < NUM_SIGMA_POINTS; ++i)
  {
    const M_by_1 diff = meas_sigma_points_[i] - y_hat_;
    Pyy_ += sigma_weights_cov_[i] * diff * diff.transpose();
  }

  // Calculate the cross covariance between the state and expected measurement
  // (would love to have Python's zip(...) functionality here)
  Pxy_ = N_by_M::Zero();
  for (std::size_t i = 0; i < NUM_SIGMA_POINTS; ++i)
  {
    Pxy_ += sigma_weights_cov_[i] * (sigma_points_[i] - x_) *
            (meas_sigma_points_[i] - y_hat_).transpose();
  }

  // Calculate the Kalman gain and innovation
  K_ = Pxy_ * Pyy_.inverse();
  innovation_ = y_ - y_hat_;

  // Update the state mean and covariance
  x_ = x_ + STATE(K_ * innovation_);
  P_ -= K_ * Pyy_ * K_.transpose();
}

template <typename STATE, typename MEAS>
template <typename MEAS_MODEL, typename... PARAMS>
void UKF<STATE, MEAS>::correct(const MEAS_MODEL& meas_model, MEAS meas,
                               PARAMS... params)
{
  measurement(std::move(meas));
  correct(meas_model, params...);
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::generate_sigma_points()
{
  // Calculate the (weighted) matrix square root of the state covariance
  // matrix
  cholesky_.compute(P_);
  const N_by_N& sqrt_P = eta_ * cholesky_.matrixL().toDenseMatrix();

  // First sigma point is the current state mean
  sigma_points_[0] = x_;

  // Next N sigma points are the current state mean perturbed by the columns
  // of sqrt_P, and the N sigma points after that are the same perturbations
  // but negated
  for (std::size_t i = 0; i < N; ++i)
  {
    const N_by_1& perturb = sqrt_P.col(i);
    sigma_points_[i + 1] = x_ + STATE(perturb);
    const N_by_1& neg_perturb = -perturb;
    sigma_points_[N + i + 1] = x_ + STATE(neg_perturb);
  }
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::state(const STATE& state)
{
  x_ = state;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::state(STATE&& state)
{
  x_ = std::move(state);
}

template <typename STATE, typename MEAS>
const STATE& UKF<STATE, MEAS>::state() const
{
  return x_;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::measurement(const MEAS& measurement)
{
  y_ = measurement;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::measurement(MEAS&& measurement)
{
  y_ = std::move(measurement);
}

template <typename STATE, typename MEAS>
const MEAS& UKF<STATE, MEAS>::measurement() const
{
  return y_;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::state_covariance(const N_by_N& state_covariance)
{
  P_ = state_covariance;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::state_covariance(N_by_N&& state_covariance)
{
  P_ = std::move(state_covariance);
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::N_by_N& UKF<STATE, MEAS>::state_covariance()
    const
{
  return P_;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::process_covariance(const N_by_N& process_covariance)
{
  Q_ = process_covariance;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::process_covariance(N_by_N&& process_covariance)
{
  Q_ = std::move(process_covariance);
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::N_by_N& UKF<STATE, MEAS>::process_covariance()
    const
{
  return Q_;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::measurement_covariance(
    const M_by_M& measurement_covariance)
{
  R_ = measurement_covariance;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::measurement_covariance(M_by_M&& measurement_covariance)
{
  R_ = std::move(measurement_covariance);
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::M_by_M&
UKF<STATE, MEAS>::measurement_covariance() const
{
  return R_;
}

template <typename STATE, typename MEAS>
const MEAS& UKF<STATE, MEAS>::expected_measurement() const
{
  return y_hat_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::M_by_M&
UKF<STATE, MEAS>::expected_measurement_covariance() const
{
  return Pyy_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::N_by_M& UKF<STATE, MEAS>::cross_covariance()
    const
{
  return Pxy_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::N_by_M& UKF<STATE, MEAS>::kalman_gain() const
{
  return K_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::M_by_1& UKF<STATE, MEAS>::innovation() const
{
  return innovation_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::SigmaPoints& UKF<STATE, MEAS>::sigma_points()
    const
{
  return sigma_points_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::MeasurementSigmaPoints&
UKF<STATE, MEAS>::measurement_sigma_points() const
{
  return meas_sigma_points_;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::weight_coefficients(double alpha, double beta,
                                           double kappa)
{
  alpha_ = alpha;
  beta_ = beta;
  kappa_ = kappa;
  calculate_weights();
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::SigmaWeights&
UKF<STATE, MEAS>::mean_sigma_weights() const
{
  return sigma_weights_mean_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::SigmaWeights&
UKF<STATE, MEAS>::covariance_sigma_weights() const
{
  return sigma_weights_cov_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::StateMeanFunction&
UKF<STATE, MEAS>::state_mean_function() const
{
  return state_mean_function_;
}

template <typename STATE, typename MEAS>
const typename UKF<STATE, MEAS>::MeasurementMeanFunction&
UKF<STATE, MEAS>::measurement_mean_function() const
{
  return meas_mean_function_;
}

template <typename STATE, typename MEAS>
void UKF<STATE, MEAS>::calculate_weights()
{
  lambda_ = alpha_ * alpha_ * (N + kappa_) - N;

  assert(N + lambda_ > 1e-6);

  eta_ = std::sqrt(N + lambda_);

  const auto w_mean_0 = lambda_ / (N + lambda_);
  const auto w_cov_0 = w_mean_0 + (1.0 - alpha_ * alpha_ + beta_);
  const auto w_i = 1.0 / (2.0 * (N + lambda_));

  sigma_weights_mean_[0] = w_mean_0;
  sigma_weights_cov_[0] = w_cov_0;

  for (std::size_t i = 1; i < NUM_SIGMA_POINTS; ++i)
  {
    sigma_weights_mean_[i] = w_i;
    sigma_weights_cov_[i] = w_i;
  }
}
} // namespace unscented

#endif
