/**
 * @file dg_to_ros.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "ros_entities/dg_to_ros.hpp"

namespace dynamic_graph
{
const char* DgToRos<double>::signalTypeName = "Double";
const char* DgToRos<unsigned int>::signalTypeName = "Unsigned";
const char* DgToRos<Matrix>::signalTypeName = "Matrix";
const char* DgToRos<Vector>::signalTypeName = "Vector";
const char* DgToRos<specific::Vector3>::signalTypeName = "Vector3";
const char* DgToRos<MatrixHomogeneous>::signalTypeName = "MatrixHomo";
const char* DgToRos<specific::Twist>::signalTypeName = "Twist";
const char* DgToRos<std::pair<specific::Vector3, Vector> >::signalTypeName =
    "Vector3Stamped";
const char* DgToRos<std::pair<MatrixHomogeneous, Vector> >::signalTypeName =
    "MatrixHomo";
const char* DgToRos<std::pair<specific::Twist, Vector> >::signalTypeName =
    "Twist";

}  // namespace dynamic_graph
