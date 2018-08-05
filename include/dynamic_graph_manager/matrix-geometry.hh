/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_MATRIX_GEOMETRY_H__
#define __SOT_MATRIX_GEOMETRY_H__


/* --- Matrix --- */
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/eigen-io.h>

#  define MRAWDATA(x) x.data()

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamic_graph {
    typedef Eigen::Transform<double,3, Eigen::Affine> MatrixHomogeneous;
    typedef Eigen::Matrix<double,3,3> MatrixRotation;
    typedef Eigen::AngleAxis<double> VectorUTheta;
    typedef Eigen::Quaternion<double> VectorQuaternion;
    typedef Eigen::Vector3d VectorRotation;
    typedef Eigen::Vector3d VectorRollPitchYaw;
    typedef Eigen::Matrix<double,6,6> MatrixForce;
    typedef Eigen::Matrix<double,6,6> MatrixTwist;

    inline void buildFrom (const MatrixHomogeneous& MH, MatrixTwist& MT) {
      
      Eigen::Vector3d _t = MH.translation();
      MatrixRotation R(MH.linear());
      Eigen::Matrix3d Tx;
      Tx << 0, -_t(2), _t(1),
	_t(2), 0, -_t(0),
	-_t(1), _t(0), 0;
      Eigen::Matrix3d sk; sk = Tx*R;
      
      MT.block<3,3>(0,0) = R;
      MT.block<3,3>(0,3) = sk;
      MT.block<3,3>(3,0).setZero();
      MT.block<3,3>(3,3) = R;
    }
} // namespace dynamicgraph

 
#endif /* #ifndef __SOT_MATRIX_GEOMETRY_H__ */
