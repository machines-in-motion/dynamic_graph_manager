/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         yaml_eigen.h

 \author       Alexander Herzog
 \date         Feb 27, 2015

 *********************************************************************/

#pragma once

#include <type_traits>
#include <iomanip>
#include <Eigen/Eigen>
//#include "robot_math/movable_eigen_matrix.h"
#include <yaml-cpp/yaml.h>

namespace robot_math{
template<class T>
struct MovableEigenVector;
template<class T>
struct MovableEigenMatrix;
}

namespace YAML {

template<class Scalar, int Rows, int Cols, int Align, int RowsAtCompileTime, int ColsAtCompileTime>
struct EigenVectorConverter{
  typedef Eigen::Matrix<Scalar,  Rows,  Cols,  Align,  RowsAtCompileTime,  ColsAtCompileTime> VectorType;

  static void resize_if_needed(int rows, int cols, VectorType& rhs){
    if(rhs.size() != rows*cols)
    {
      if (VectorType::SizeAtCompileTime == Eigen::Dynamic &&
          VectorType::MaxSizeAtCompileTime == Eigen::Dynamic)
      {
        rhs.resize(rows*cols);
      }else
      {
        std::ostringstream error;
        error << "ERROR: The fixed sized vector of size (" << rhs.size()
              << ") is of different size than the input yaml data vector of "
              << "size (" << rows * cols << ")." ;
        throw(std::runtime_error(error.str()));
      }
    }
  }

  static Scalar& access_element(int r, int c, VectorType& rhs)
  {
    return rhs(r+c);
  }
};

template<class Scalar, int Rows, int Cols, int Align, int RowsAtCompileTime, int ColsAtCompileTime>
struct EigenMatrixConverter{
  typedef Eigen::Matrix<Scalar,  Rows,  Cols,  Align,  RowsAtCompileTime,  ColsAtCompileTime> MatrixType;

  static void resize_if_needed(int rows, int cols, MatrixType& rhs){
    if(rhs.rows() != rows || rhs.cols() != cols)
    {
      if (MatrixType::SizeAtCompileTime == Eigen::Dynamic &&
          MatrixType::MaxSizeAtCompileTime == Eigen::Dynamic)
      {
        rhs.resize(rows, cols);
      }else
      {
        std::ostringstream error;
        error << "ERROR: The fixed sized matrix of dim (" << rhs.rows() << ","
              << rhs.cols() << ") is of different dim than the input yaml "
              << "data matrix of dim (" << rows << "," << cols << ")." ;
        throw(std::runtime_error(error.str()));
      }
    }
  }


  static Scalar& access_element(int r, int c, MatrixType& rhs){
    return rhs(r, c);
  }
};

template<class Scalar, int Rows, int Cols, int Align, int RowsAtCompileTime, int ColsAtCompileTime>
struct convert<Eigen::Matrix<Scalar,  Rows,  Cols,  Align,  RowsAtCompileTime,  ColsAtCompileTime> >{
  typedef Eigen::Matrix<Scalar,  Rows,  Cols,  Align,  RowsAtCompileTime,  ColsAtCompileTime> Eigen_Type_;
  const static bool is_vector_type_ = Rows == 1 || Cols == 1;
  typedef typename std::conditional<is_vector_type_,
      EigenVectorConverter<Scalar, Rows, Cols, Align, RowsAtCompileTime, ColsAtCompileTime>,
      EigenMatrixConverter<Scalar, Rows, Cols, Align, RowsAtCompileTime, ColsAtCompileTime>>::type Converter_;

  static Node encode(const Eigen_Type_& rhs) {
    Eigen::IOFormat yaml_format(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "[", "]", "[", "]");
    std::stringstream ss;
    ss << std::setprecision (std::numeric_limits<double>::digits10 + 1) << rhs.format(yaml_format);
    return Load(ss.str());
  }

  static bool decode_2d(const Node& node, Eigen_Type_& rhs){
    const int n_rows = node.size();
    const int n_cols = node[0].size();
    Converter_::resize_if_needed(n_rows, n_cols, rhs);

    for (int r=0;r<n_rows;++r){
      const Node& yaml_row = node[r];
      if(yaml_row.size() != n_cols)
        return false;
      for (int c=0;c<n_cols;++c)
        Converter_::access_element(r, c, rhs) = yaml_row[c].as<Scalar>();
    }

    return true;
  }

  static bool decode_1d(const Node& node, Eigen_Type_& rhs){
    const int n_size = node.size();
    Converter_::resize_if_needed(n_size, 1, rhs);
    for (int r=0;r<n_size;++r)
      Converter_::access_element(r, 0, rhs) = node[r].as<Scalar>();
    return true;
  };

  static bool decode(const Node& node, Eigen_Type_& rhs){
    if(!node.IsSequence())
      return false;
    else if(!node[0].IsSequence())
      return decode_1d(node, rhs);
    else
      return decode_2d(node, rhs);
  }

};


template<class T>
struct convert<robot_math::MovableEigenVector<T>> {
  static Node encode(const robot_math::MovableEigenVector<T>& mp){
    return convert<T>::encode(static_cast<const T&>(mp));
  }
  static bool decode(const Node& node, robot_math::MovableEigenVector<T>& mp){
    return convert<T>::decode(node, static_cast<T&>(mp));
  }
};

template<>
struct convert<Eigen::Quaterniond> {
  static Node encode(const Eigen::Quaterniond& mp){
    return convert<Eigen::Vector4d>::encode(Eigen::Vector4d(mp.w(), mp.x(), mp.y(), mp.z()));
  }
  static bool decode(const Node& node, Eigen::Quaterniond& mp){
    Eigen::Vector4d v4;
    if(!convert<Eigen::Vector4d>::decode(node, v4)) return false;
    mp.w() = v4[0];
    mp.x() = v4[1];
    mp.y() = v4[2];
    mp.z() = v4[3];
    return true;
  }
};

} // namespace YAML


