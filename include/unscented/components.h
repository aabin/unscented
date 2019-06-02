#include <Eigen/Dense>

namespace unscented
{
///////////////////////////////////////////////////////////////////////////////
// Vectors
///////////////////////////////////////////////////////////////////////////////
using Vector1 = Eigen::Matrix<double, 1, 1>;
using Vector2 = Eigen::Matrix<double, 2, 1>;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using Vector4 = Eigen::Matrix<double, 4, 1>;
using Vector5 = Eigen::Matrix<double, 5, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Vector8 = Eigen::Matrix<double, 8, 1>;
using Vector9 = Eigen::Matrix<double, 9, 1>;
using Vector10 = Eigen::Matrix<double, 10, 1>;
using Vector11 = Eigen::Matrix<double, 11, 1>;
using Vector12 = Eigen::Matrix<double, 12, 1>;
using Vector13 = Eigen::Matrix<double, 13, 1>;
using Vector14 = Eigen::Matrix<double, 14, 1>;
using Vector15 = Eigen::Matrix<double, 15, 1>;
using Vector16 = Eigen::Matrix<double, 16, 1>;
using Vector17 = Eigen::Matrix<double, 17, 1>;
using Vector18 = Eigen::Matrix<double, 18, 1>;
using Vector19 = Eigen::Matrix<double, 19, 1>;
using Vector20 = Eigen::Matrix<double, 20, 1>;

template <std::size_t DIM, std::size_t NUM_VECS>
Eigen::Matrix<double, DIM, 1> vector_mean_function(
    const std::array<Eigen::Matrix<double, DIM, 1>, NUM_VECS>& vectors,
    const std::array<double, NUM_VECS>& weights);

///////////////////////////////////////////////////////////////////////////////
// UnitComplex
///////////////////////////////////////////////////////////////////////////////
struct UnitComplex
{
  static constexpr std::size_t DOF = 1;

  static UnitComplex Zero();

  explicit UnitComplex(const Vector1& vec);

  explicit UnitComplex(double angle);

  UnitComplex(double a_in, double b_in);

  double angle() const;

  double a;

  double b;
};

UnitComplex operator+(const UnitComplex& lhs, const UnitComplex& rhs);

Vector1 operator-(const UnitComplex& lhs, const UnitComplex& rhs);

UnitComplex unit_complex_mean_function(
    const std::array<UnitComplex, 2 * UnitComplex::DOF + 1>& states,
    const std::array<double, 2 * UnitComplex::DOF + 1>& weights);

/////////////////////////////////////////////////////////////////////////////////
//// SO2
/////////////////////////////////////////////////////////////////////////////////
//struct SO2
//{
//  static constexpr std::size_t DOF = 1;

//  static SO2 Zero();

//  explicit SO2(const Vector<DOF>& vec);

//  explicit SO2(const Eigen::Matrix2d& rotation_matrix);

//  explicit SO2(const UnitComplex& unit_complex);

//  explicit SO2(double angle);

//  UnitComplex unit_complex() const;

//  double angle() const;

//  Eigen::Matrix2d rotation_matrix;
//};

//SO2 operator+(const SO2& lhs, const SO2& rhs);

//Vector<SO2::DOF> operator-(const SO2& lhs, const SO2& rhs);

//UnitComplex SO2_mean_function(
//    const std::array<UnitComplex, 2 * UnitComplex::DOF + 1>& states,
//    const std::array<double, 2 * UnitComplex::DOF + 1>& weights);

/////////////////////////////////////////////////////////////////////////////////
//// SE(2)
/////////////////////////////////////////////////////////////////////////////////
//struct SE2
//{
//  static constexpr std::size_t DOF = 3;

//  static SE2 Zero();

//  SE2() = default;

//  SE2(const Vector<2>& position, const SO2& rotation);

//  SE2(const Vector<2>& position, const UnitComplex& unit_complex);

//  SE2(const Vector<2>& position, double angle);

//  SE2(const Vector<2>& position, double a, double b);

//  SE2(double x, double y, const SO2& rotation);

//  SE2(double x, double y, const UnitComplex& unit_complex);

//  SE2(double x, double y, double angle);

//  SE2(double x, double y, double a, double b);

//  explicit SE2(const Eigen::Affine2d& affine);

//  explicit SE2(const Vector<3>& vec);

//  Eigen::Affine2d affine{Eigen::Affine2d::Identity()};
//};

//SE2 operator+(const SE2& lhs, const SE2& rhs);

//Vector<3> operator-(const SE2& lhs, const SE2& rhs);

/////////////////////////////////////////////////////////////////////////////////
//// UnitQuaternion
/////////////////////////////////////////////////////////////////////////////////
//struct SO3;

//struct UnitQuaternion
//{
//  static constexpr std::size_t DOF = 3;

//  UnitQuaternion() = default;

//  UnitQuaternion(double w, double x, double y, double z);

//  explicit UnitQuaternion(const Eigen::Quaterniond& q);

//  explicit UnitQuaternion(const SO3& so3);

//  explicit UnitQuaternion(const Eigen::Matrix3d& rotation_matrix);

//  explicit UnitQuaternion(const Vector<3>& vec);

//  Eigen::Quaterniond q;
//};

//UnitQuaternion operator+(const UnitQuaternion& lhs, const UnitQuaternion& rhs);

//Vector<3> operator-(const UnitQuaternion& lhs, const UnitQuaternion& rhs);

/////////////////////////////////////////////////////////////////////////////////
//// SO3
/////////////////////////////////////////////////////////////////////////////////
//struct SO3
//{
//  static constexpr std::size_t DOF = 3;

//  SO3() = default;

//  explicit SO3(const Eigen::Matrix3d& rotation_matrix);

//  explicit SO3(const UnitQuaternion& q);

//  explicit SO3(const Eigen::Quaterniond& q);

//  explicit SO3(const Vector<3>& vec);

//  Eigen::Matrix2d rotation_matrix;
//};

//SO3 operator+(const SO3& lhs, const SO3& rhs);

//Vector<3> operator-(const SO3& lhs, const SO3& rhs);

/////////////////////////////////////////////////////////////////////////////////
//// SE(3)
/////////////////////////////////////////////////////////////////////////////////
//struct SE3
//{
//  static constexpr std::size_t DOF = 6;

//  SE3() = default;

//  SE3(const Vector<3>& position, const SO3& rotation);

//  SE3(const Vector<3>& position, const UnitQuaternion& q);

//  SE3(const Vector<3>& position, const Eigen::Quaterniond& q);

//  SE3(const Vector<3>& position, const Eigen::Matrix3d& rotation_matrix);

//  SE3(double x, double y, double z, const SO3& rotation);

//  SE3(double x, double y, double z, const UnitQuaternion& q);

//  SE3(double x, double y, double z, const Eigen::Quaterniond& q);

//  SE3(double x, double y, double z, const Eigen::Matrix3d& rotation_matrix);

//  explicit SE3(const Eigen::Affine3d& affine);

//  explicit SE3(const Vector<6>& vec);

//  Eigen::Affine3d affine{Eigen::Affine3d::Identity()};
//};

//SE3 operator+(const SE3& lhs, const SE3& rhs);

//Vector<6> operator-(const SE3& lhs, const SE3& rhs);
} // namespace unscented