#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H


#define RAD2DEG (0.01745329251994329576923690768489)
// constexpr size_t MAX_DOF=50;

#include <Eigen/Dense>

#define GRAVITY 9.80665
#define MAX_DOF 50U
#define DEG2RAD 1/RAD2DEG

namespace Eigen
{

// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
  typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
  typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

typedef double	rScalar;

EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 8, 8)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 12, 12)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 28, 28)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 30, 30)

// typedef Transform<rScalar, 3, Eigen::Isometry> HTransform;  // typedef Transform< double, 3, Isometry > 	Eigen::Isometry3d

typedef Matrix<rScalar, 1, 3>	Matrix1x3d;
typedef Matrix<rScalar, 1, 4>	Matrix1x4d;
typedef Matrix<rScalar, 4, 3>	Matrix4x3d;
typedef Matrix<rScalar, 6, 3>	Matrix6x3d;
typedef Matrix<rScalar, 6, 7>	Matrix6x7d;
typedef Matrix<rScalar, 8, 4>	Matrix8x4d;
typedef Matrix<rScalar, -1, 1, 0, MAX_DOF, 1> VectorJXd;
typedef Matrix<rScalar, -1, 1, 0, 12, 1> VectorLXd; //Leg IK
typedef Matrix<rScalar, -1, -1, 0, MAX_DOF, MAX_DOF> MatrixJXd;

//Complex
typedef Matrix<std::complex<double>,8,4> Matrix8x4cd;

}

namespace DyrosMath
{

//constexpr double GRAVITY {9.80665};
//constexpr double DEG2RAD {};

static double cubic(double time,     ///< Current time
             double time_0,   ///< Start time
             double time_f,   ///< End time
             double x_0,      ///< Start state
             double x_f,      ///< End state
             double x_dot_0,  ///< Start state dot
             double x_dot_f   ///< End state dot
             )
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_0;
  }
  else if (time > time_f)
  {
    x_t = x_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

        + (3 * total_x / total_time2
           - 2 * x_dot_0 / total_time
           - x_dot_f / total_time)
        * elapsed_time * elapsed_time

        + (-2 * total_x / total_time3 +
           (x_dot_0 + x_dot_f) / total_time2)
        * elapsed_time * elapsed_time * elapsed_time;
  }

  return x_t;
}

static double cubicDot(double time,     ///< Current time
             double time_0,   ///< Start time
             double time_f,   ///< End time
             double x_0,      ///< Start state
             double x_f,      ///< End state
             double x_dot_0,  ///< Start state dot
             double x_dot_f,   ///< End state dot
             double hz         ///< control frequency
             )
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_0;
  }
  else if (time > time_f)
  {
    x_t = x_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_t = x_dot_0

        + 2*(3 * total_x / total_time2
           - 2 * x_dot_0 / total_time
           - x_dot_f / total_time)
        * elapsed_time

        + 3*(-2 * total_x / total_time3 +
           (x_dot_0 + x_dot_f) / total_time2)
        * elapsed_time * elapsed_time;
  }

  return x_t;
}

template <int N>
static Eigen::Matrix<double, N, 1> cubicVector(double time,     ///< Current time
                                                double time_0,   ///< Start time
                                                double time_f,   ///< End time
                                                Eigen::Matrix<double, N, 1> x_0,      ///< Start state
                                                Eigen::Matrix<double, N, 1> x_f,      ///< End state
                                                Eigen::Matrix<double, N, 1> x_dot_0,  ///< Start state dot
                                                Eigen::Matrix<double, N, 1> x_dot_f   ///< End state dot
    )
{

  Eigen::Matrix<double, N, 1> res;
  for (unsigned int i=0; i<N; i++)
  {
    res(i) = cubic(time, time_0, time_f, x_0(i), x_f(i), x_dot_0(i), x_dot_f(i));
  }
  return res;
}

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++) {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5* phi;

  return phi;
}

static Eigen::Isometry3d multiplyIsometry3d(Eigen::Isometry3d A,
                                      Eigen::Isometry3d B)
{
  Eigen::Isometry3d AB;

  AB.linear() = A.linear()*B.linear();
  AB.translation() = A.linear()*B.translation() + A.translation();
  return AB;
}

static Eigen::Isometry3d inverseIsometry3d(Eigen::Isometry3d A)
{
  Eigen::Isometry3d A_inv;

  A_inv.linear() = A.linear().transpose();
  A_inv.translation() = -A.linear().transpose()*A.translation();
  return A_inv;
}

static Eigen::Matrix3d rotateWithZ(double yaw_angle)
{
  Eigen::Matrix3d rotate_wth_z(3, 3);

  rotate_wth_z(0, 0) = cos(yaw_angle);
  rotate_wth_z(1, 0) = sin(yaw_angle);
  rotate_wth_z(2, 0) = 0.0;

  rotate_wth_z(0, 1) = -sin(yaw_angle);
  rotate_wth_z(1, 1) = cos(yaw_angle);
  rotate_wth_z(2, 1) = 0.0;

  rotate_wth_z(0, 2) = 0.0;
  rotate_wth_z(1, 2) = 0.0;
  rotate_wth_z(2, 2) = 1.0;

  return rotate_wth_z;
}

static Eigen::Matrix3d rotateWithY(double pitch_angle)
{
  Eigen::Matrix3d rotate_wth_y(3, 3);

  rotate_wth_y(0, 0) = cos(pitch_angle);
  rotate_wth_y(1, 0) = 0.0;
  rotate_wth_y(2, 0) = -sin(pitch_angle);

  rotate_wth_y(0, 1) = 0.0;
  rotate_wth_y(1, 1) = 1.0;
  rotate_wth_y(2, 1) = 0.0;

  rotate_wth_y(0, 2) = sin(pitch_angle);
  rotate_wth_y(1, 2) = 0.0;
  rotate_wth_y(2, 2) = cos(pitch_angle);

  return rotate_wth_y;
}

static Eigen::Matrix3d rotateWithX(double roll_angle)
{
  Eigen::Matrix3d rotate_wth_x(3, 3);

  rotate_wth_x(0, 0) = 1.0;
  rotate_wth_x(1, 0) = 0.0;
  rotate_wth_x(2, 0) = 0.0;

  rotate_wth_x(0, 1) = 0.0;
  rotate_wth_x(1, 1) = cos(roll_angle);
  rotate_wth_x(2, 1) = sin(roll_angle);

  rotate_wth_x(0, 2) = 0.0;
  rotate_wth_x(1, 2) = -sin(roll_angle);
  rotate_wth_x(2, 2) = cos(roll_angle);

  return rotate_wth_x;
}

static Eigen::Vector3d rot2Euler(Eigen::Matrix3d Rot)
{
    double beta;
    Eigen::Vector3d angle;
    beta = -asin(Rot(2,0));

    if(abs(beta) < 90*DEG2RAD)
        beta = beta;
    else
        beta = 180*DEG2RAD-beta;

    angle(0) = atan2(Rot(2,1),Rot(2,2)+1E-37); //roll
    angle(2) = atan2(Rot(1,0),Rot(0,0)+1E-37); //pitch
    angle(1) = beta; //yaw

    return angle;
}

template <typename _Matrix_Type_>
_Matrix_Type_ pinv(const _Matrix_Type_ &a, double epsilon =std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

static void floatGyroframe(Eigen::Isometry3d trunk, Eigen::Isometry3d reference, Eigen::Isometry3d new_trunk)
{
  Eigen::Vector3d rpy_ang;
  rpy_ang = DyrosMath::rot2Euler(reference.linear());

  Eigen::Matrix3d temp;
  temp = DyrosMath::rotateWithZ(-rpy_ang(2));

  new_trunk.linear() = temp*trunk.linear();
  new_trunk.translation() = temp*(trunk.translation() - reference.translation());
}


}

#endif
