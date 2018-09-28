#include "sot_to_ros.hh"

namespace dynamicgraph
{

  const char* SotToRos<double>::signalTypeName = "Double";
  const char* SotToRos<unsigned int>::signalTypeName = "Unsigned";
  const char* SotToRos<Matrix>::signalTypeName = "Matrix";
  const char* SotToRos<Vector>::signalTypeName = "Vector";
  const char* SotToRos<specific::Vector3>::signalTypeName = "Vector3";
  const char* SotToRos<sot::MatrixHomogeneous>::signalTypeName = "MatrixHomo";
  const char* SotToRos<specific::Twist>::signalTypeName = "Twist";
  const char* SotToRos
  <std::pair<specific::Vector3, Vector> >::signalTypeName
  = "Vector3Stamped";
  const char* SotToRos
  <std::pair<sot::MatrixHomogeneous, Vector> >::signalTypeName
  = "MatrixHomo";
  const char* SotToRos
  <std::pair<specific::Twist, Vector> >::signalTypeName
  = "Twist";

} // end of namespace dynamicgraph.
