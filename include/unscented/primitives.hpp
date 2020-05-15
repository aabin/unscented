#ifndef UNSCENTED_PRIMITIVES_HPP
#define UNSCENTED_PRIMITIVES_HPP

#include "unscented/primitives.h"

namespace unscented
{
///////////////////////////////////////////////////////////////////////////////
// Rotation2d
///////////////////////////////////////////////////////////////////////////////
Rotation2d::Rotation2d(const Vector<1>& vec) : Rotation2d(vec(0))
{
}

Rotation2d::Rotation2d(double angle)
  : Rotation2d(std::cos(angle), std::sin(angle))
{
}

Rotation2d::Rotation2d(const Matrix<2, 2>& rotation_matrix)
  : Rotation2d(rotation_matrix(0, 0), rotation_matrix(1, 0))
{
}

Rotation2d::Rotation2d(const Vector<2>& unit_complex)
  : Rotation2d(unit_complex(0), unit_complex(1))
{
}

Rotation2d::Rotation2d(double a_in, double b_in) : a(a_in), b(b_in)
{
  static const double EPS = 1e-6;
  const auto sq_norm = a * a + b * b;
  if (std::abs(1 - sq_norm) > EPS)
  {
    const auto norm = std::sqrt(sq_norm);
    a /= norm;
    b /= norm;
  }
}

double Rotation2d::angle() const
{
  return std::atan2(b, a);
}

Matrix<2, 2> Rotation2d::rotation_matrix() const
{
  Matrix<2, 2> ret;
  ret << a, -b, b, a; // a is cos(angle), b is sin(angle)
  return ret;
}

Vector<2> Rotation2d::unit_complex() const
{
  return {a, b};
}

Rotation2d operator+(const Rotation2d& lhs, const Rotation2d& rhs)
{
  return Rotation2d(lhs.angle() + rhs.angle());
}

Vector<1> operator-(const Rotation2d& lhs, const Rotation2d& rhs)
{
  return Vector<1>(Rotation2d(rhs.angle() - lhs.angle()).angle());
}

template <std::size_t SIZE>
Rotation2d mean_function(const std::array<Rotation2d, SIZE>& rotations,
                         const std::array<double, SIZE>& weights)
{
  Vector<2> mean_unit_complex;
  for (std::size_t i = 0; i < SIZE; ++i)
  {
    mean_unit_complex =
        mean_unit_complex + weights[i] * rotations[i].unit_complex();
  }
  return Rotation2d(mean_unit_complex);
}

} // namespace unscented

#endif
