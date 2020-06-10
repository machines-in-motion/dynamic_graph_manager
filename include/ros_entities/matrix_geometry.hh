/**
 * @file matrix_geometry.hh
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#ifndef MATRIX_GEOMETRY_HH
#define MATRIX_GEOMETRY_HH

#include <dynamic-graph/eigen-io.h>
#include <dynamic-graph/linear-algebra.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define MRAWDATA(x) x.data()

namespace dynamic_graph
{
typedef Eigen::Transform<double, 3, Eigen::Affine> MatrixHomogeneous;
typedef Eigen::Matrix<double, 3, 3> MatrixRotation;
typedef Eigen::AngleAxis<double> VectorUTheta;
typedef Eigen::Quaternion<double> VectorQuaternion;
typedef Eigen::Vector3d VectorRotation;
typedef Eigen::Vector3d VectorRollPitchYaw;
typedef Eigen::Matrix<double, 6, 6> MatrixForce;
typedef Eigen::Matrix<double, 6, 6> MatrixTwist;

inline void buildFrom(const MatrixHomogeneous& MH, MatrixTwist& MT)
{
    Eigen::Vector3d _t = MH.translation();
    MatrixRotation R(MH.linear());
    Eigen::Matrix3d Tx;
    Tx << 0, -_t(2), _t(1), _t(2), 0, -_t(0), -_t(1), _t(0), 0;
    Eigen::Matrix3d sk;
    sk = Tx * R;

    MT.block<3, 3>(0, 0) = R;
    MT.block<3, 3>(0, 3) = sk;
    MT.block<3, 3>(3, 0).setZero();
    MT.block<3, 3>(3, 3) = R;
}
}  // namespace dynamic_graph

#endif  // MATRIX_GEOMETRY_HH
