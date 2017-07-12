#pragma once

#include <eigen3/Eigen/Dense>

#define Gravity	9.81
#define DEGREE	(0.01745329251994329576923690768489)

namespace Eigen
{
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
/** \ingroup matrixtypedefs */                                    \
typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

typedef double	rScalar;

EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
typedef Transform<rScalar, 3, Eigen::Isometry> HTransform;
typedef Matrix<rScalar, 6, 3>	Matrix6x3d;

}

