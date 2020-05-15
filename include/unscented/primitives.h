#ifndef UNSCENTED_PRIMITIVES_H
#define UNSCENTED_PRIMITIVES_H

#include <Eigen/Dense>

namespace unscented
{
///////////////////////////////////////////////////////////////////////////////
// Vectors and Matrices
///////////////////////////////////////////////////////////////////////////////
template <std::size_t DIM>
using Vector = Eigen::Matrix<double, DIM, 1>;

template <std::size_t ROWS, std::size_t COLS>
using Matrix = Eigen::Matrix<double, ROWS, COLS>;

///////////////////////////////////////////////////////////////////////////////
// Rotation2d
///////////////////////////////////////////////////////////////////////////////
class Rotation2d
{
public:
  static constexpr std::size_t DOF = 1;

  Rotation2d() = default;

  explicit Rotation2d(const Vector<1>& vec);

  explicit Rotation2d(double angle);

  explicit Rotation2d(const Matrix<2, 2>& rotation_matrix);

  explicit Rotation2d(const Vector<2>& unit_complex);

  Rotation2d(double a_in, double b_in);

  double angle() const;

  Matrix<2, 2> rotation_matrix() const;

  Vector<2> unit_complex() const;

private:
  double a{1.0};

  double b{0.0};
};

Rotation2d operator+(const Rotation2d& lhs, const Rotation2d& rhs);

Vector<1> operator-(const Rotation2d& lhs, const Rotation2d& rhs);

template <std::size_t SIZE>
Rotation2d mean_function(const std::array<Rotation2d, SIZE>& rotations,
                         const std::array<double, SIZE>& weights);

} // namespace unscented

#endif
