#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H


#define DEG2RAD (0.01745329251994329576923690768489)
// constexpr size_t MAX_DOF=50;

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#define GRAVITY 9.80665
#define MAX_DOF 50U
#define RAD2DEG 1/DEG2RAD


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
EIGEN_MAKE_TYPEDEFS(rScalar, d, 18, 18)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 28, 28)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 30, 30)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 32, 32)

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

static Eigen::Matrix3d skew(Eigen::Vector3d src)
{
    Eigen::Matrix3d skew;
    skew.setZero();
    skew(0, 1) = -src[2];
    skew(0, 2) = src[1];
    skew(1, 0) = src[2];
    skew(1, 2) = -src[0];
    skew(2, 0) = -src[1];
    skew(2, 1) = src[0];

    return skew;
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

// Original Paper
// Kang, I. G., and F. C. Park.
// "Cubic spline algorithms for orientation interpolation."
// International journal for numerical methods in engineering 46.1 (1999): 45-64.
const static Eigen::Matrix3d rotationCubic(double time,
                                     double time_0,
                                     double time_f,
                                     const Eigen::Matrix3d &rotation_0,
                                     const Eigen::Matrix3d &rotation_f)
{
  if(time >= time_f)
  {
    return rotation_f;
  }
  else if(time < time_0)
  {
    return rotation_0;
  }
  double tau = cubic(time,time_0,time_f,0,1,0,0);
  Eigen::Matrix3d rot_scaler_skew;
  rot_scaler_skew = (rotation_0.transpose() * rotation_f).log();
  //rot_scaler_skew = rot_scaler_skew.log();
  /*
  Eigen::Matrix3d rotation_exp;
  Eigen::Vector3d a1, b1, c1, r1;
  r1(0) = rotation_temp(2,1);
  r1(1) = rotation_temp(0,2);
  r1(2) = rotation_temp(1,0);
  c1.setZero(); // angular velocity at t0 --> Zero
  b1.setZero(); // angular acceleration at t0 --> Zero
  a1 = r1 - b1 - c1;
  //double tau = (time - time_0) / (time_f-time_0);
  double tau2 = tau*tau;
  double tau3 = tau2*tau;
  //Eigen::Vector3d exp_vector = (a1*tau3+b1*tau2+c1*tau);
  Eigen::Vector3d exp_vector = (a1*tau);
  rotation_exp.setZero();
  rotation_exp(0,1) = -exp_vector(2);
  rotation_exp(0,2) =  exp_vector(1);
  rotation_exp(1,0) =  exp_vector(2);
  rotation_exp(1,2) = -exp_vector(0);
  rotation_exp(2,0) = -exp_vector(1);
  rotation_exp(2,1) =  exp_vector(0);

  */
  //Eigen::Matrix3d result = rotation_0 * rotation_exp.exp();
  Eigen::Matrix3d result = rotation_0 * (rot_scaler_skew * tau).exp();

  return result;
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


template <int _State_Size_, int _Input_Size_>
Eigen::Matrix<double, _State_Size_, _State_Size_> discreteRiccatiEquation(
    Eigen::Matrix<double, _State_Size_, _State_Size_> a,
    Eigen::Matrix<double, _State_Size_, _Input_Size_> b,
    Eigen::Matrix<double, _Input_Size_, _Input_Size_> r,
    Eigen::Matrix<double, _State_Size_, _State_Size_> q)
{
  Eigen::Matrix4d z11, z12, z21, z22;
  z11 = a.inverse();
  z12 = a.inverse()*b*r.inverse()*b.transpose();
  z21 = q*a.inverse();
  z22 = a.transpose() + q*a.inverse()*b*r.inverse()*b.transpose();

  Eigen::Matrix<double, 2*_State_Size_, 2*_State_Size_> z;
  z.setZero();
  z.topLeftCorner(4,4) = z11;
  z.topRightCorner(4,4) = z12;
  z.bottomLeftCorner(4,4) = z21;
  z.bottomRightCorner(4,4) = z22;

  std::vector<double> eigVal_real(8);
  std::vector<double> eigVal_img(8);
  std::vector<Eigen::Vector8d> eigVec_real(8);
  std::vector<Eigen::Vector8d> eigVec_img(8);

  for(int i=0; i<8; i++)
  {
    eigVec_real[i].setZero();
    eigVec_img[i].setZero();
  }

  Eigen::Matrix<double, 2*_State_Size_, 1> deigVal_real, deigVal_img;
  Eigen::Matrix<double, 2*_State_Size_, 2*_State_Size_> deigVec_real, deigVec_img;
  deigVal_real.setZero();
  deigVal_img.setZero();
  deigVec_real.setZero();
  deigVec_img.setZero();
  deigVal_real = z.eigenvalues().real();
  deigVal_img = z.eigenvalues().imag();

  Eigen::EigenSolver<Eigen::Matrix<double, 2*_State_Size_, 2*_State_Size_>> ev(z);
  //EigenVector Solver
  //Matrix3D ones = Matrix3D::Ones(3,3);
  //EigenSolver<Matrix3D> ev(ones);
  //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ev.eigenvectors().col(1) << endl;

  for(int i=0;i<8; i++)
  {
    for(int j=0; j<8; j++)
    {
      deigVec_real(j,i) = ev.eigenvectors().col(i)(j).real();
      deigVec_img(j,i) = ev.eigenvectors().col(i)(j).imag();
    }
  }

  //Order the eigenvectors
  //move e-vectors correspnding to e-value outside the unite circle to the left

  Eigen::Matrix8x4d tempZ_real, tempZ_img;
  tempZ_real.setZero();
  tempZ_img.setZero();
  int c=0;

  for (int i=0;i<8;i++)
  {
    if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
    {
      for(int j=0; j<8; j++)
      {
        tempZ_real(j,c) = deigVec_real(j,i);
        tempZ_img(j,c) = deigVec_img(j,i);
      }
      c++;
    }
  }

  Eigen::Matrix8x4cd tempZ_comp;
  for(int i=0;i<8;i++)
  {
    for(int j=0;j<4;j++)
    {
      tempZ_comp.real()(i,j) = tempZ_real(i,j);
      tempZ_comp.imag()(i,j) = tempZ_img(i,j);
    }
  }

  Eigen::Matrix4cd U11, U21, X;
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<4;j++)
    {
      U11(i,j) = tempZ_comp(i,j);
      U21(i,j) = tempZ_comp(i+4,j);
    }
  }
  X = U21*(U11.inverse());
  Eigen::Matrix4d X_sol;
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<4;j++)
    {
      X_sol(i,j) = X.real()(i,j);
    }
  }

  return X_sol;
}

static Eigen::Vector3d legGetPhi(Eigen::Isometry3d rotation_matrix1, Eigen::Isometry3d active_r1, Eigen::Vector6d ctrl_pos_ori)
{
   Eigen::Matrix3d active_r, rotation_matrix, x_rot, y_rot, z_rot, d_rot, s1_skew, s2_skew, s3_skew;
   x_rot.setZero();
   y_rot.setZero();
   z_rot.setZero();
   d_rot.setZero();
   s1_skew.setZero();
   s2_skew.setZero();
   s3_skew.setZero();

   active_r = active_r1.linear();

   x_rot=rotateWithX(ctrl_pos_ori(3));
   y_rot=rotateWithY(ctrl_pos_ori(4));
   z_rot=rotateWithZ(ctrl_pos_ori(5));
   d_rot=active_r.inverse()*z_rot*y_rot*x_rot;

   rotation_matrix = active_r.inverse() * rotation_matrix1.linear();

   s1_skew=skew(rotation_matrix.col(0));
   s2_skew=skew(rotation_matrix.col(1));
   s3_skew=skew(rotation_matrix.col(2));

   Eigen::Vector3d s1f, s2f, s3f, phi;
   s1f.setZero();
   s2f.setZero();
   s3f.setZero();

   s1f = s1_skew * d_rot.col(0);
   s2f = s2_skew * d_rot.col(1);
   s3f = s3_skew * d_rot.col(2);

   phi = (s1f + s2f + s3f) * (-1.0/2.0);

  return phi;
}

static Eigen::Vector3d QuinticSpline(
                   double time,       ///< Current time
                   double time_0,     ///< Start time
                   double time_f,     ///< End time
                   double x_0,        ///< Start state
                   double x_dot_0,    ///< Start state dot
                   double x_ddot_0,   ///< Start state ddot
                   double x_f,        ///< End state
                   double x_dot_f,    ///< End state
                   double x_ddot_f )  ///< End state ddot
{
  double a1,a2,a3,a4,a5,a6;
  double time_s;

  Eigen::Vector3d result;

  if(time < time_0)
  {
    result << x_0, x_dot_0, x_ddot_0;
    return result;
  }
  else if (time > time_f)
  {
    result << x_f, x_dot_f, x_ddot_f;
    return result;
  }


  time_s = time_f - time_0;
  a1=x_0;
  a2=x_dot_0;
  a3=x_ddot_0/2.0;

  Eigen::Matrix3d Temp;
  Temp<<pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
        6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

  Eigen::Vector3d R_temp;
  R_temp<<x_f-x_0-x_dot_0*time_s-x_ddot_0*pow(time_s,2)/2.0,
        x_dot_f-x_dot_0-x_ddot_0*time_s,
        x_ddot_f-x_ddot_0;

  Eigen::Vector3d RES;

  RES = Temp.inverse()*R_temp;

  a4=RES(0);
  a5=RES(1);
  a6=RES(2);

  double time_fs = time - time_0;

  double position = a1+a2*pow(time_fs,1)+a3*pow(time_fs,2)+a4*pow(time_fs,3)+a5*pow(time_fs,4)+a6*pow(time_fs,5);
  double velocity = a2+2.0*a3*pow(time_fs,1)+3.0*a4*pow(time_fs,2)+4.0*a5*pow(time_fs,3)+5.0*a6*pow(time_fs,4);
  double acceleration =2.0*a3+6.0*a4*pow(time_fs,1)+12.0*a5*pow(time_fs,2)+20.0*a6*pow(time_fs,3);


  result<<position,velocity,acceleration;

  return result;
}




}
#endif
