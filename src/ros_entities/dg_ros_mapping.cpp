/**
 * @file dg_to_ros.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck
 * Gesellschaft.
 * @date 2019-05-22
 */

#include "dynamic_graph_manager/ros_entities/dg_ros_mapping.hpp"

namespace dynamic_graph_manager
{
const std::vector<std::string> DgRosTypes::type_list =
    DgRosTypes::create_type_list();

#define DG_TO_ROS_IMPL(DgRosMappingType)      \
    template <>                               \
    inline void DgRosMappingType::dg_to_ros(  \
        const DgRosMappingType::dg_t& dg_src, \
        DgRosMappingType::ros_t& ros_dst)

#define ROS_TO_DG_IMPL(DgRosMappingType)        \
    template <>                                 \
    inline void DgRosMappingType::ros_to_dg(    \
        const DgRosMappingType::ros_t& ros_src, \
        DgRosMappingType::dg_t& dg_dst)

#define DG_DEFAULT_VALUE(DgRosMappingType) \
    template <>                            \
    const DgRosMappingType::dg_t DgRosMappingType::default_value

#define DG_SIGNAL_TYPE_NAME(DgRosMappingType) \
    template <>                               \
    const std::string DgRosMappingType::signal_type_name

/*
 * Float
 */
typedef DgRosMapping<std_msgs::msg::Float64, double> DgRosMappingFloat;
DG_SIGNAL_TYPE_NAME(DgRosMappingFloat) = DgRosTypes::get_double();
DG_DEFAULT_VALUE(DgRosMappingFloat) = 0.0;
DG_TO_ROS_IMPL(DgRosMappingFloat)
{
    ros_dst.data = dg_src;
}
ROS_TO_DG_IMPL(DgRosMappingFloat)
{
    dg_dst = ros_src.data;
}

/*
 * Unsigned int
 */

typedef DgRosMapping<std_msgs::msg::UInt32, unsigned int>
    DgRosMappingUnsignedInt;
DG_SIGNAL_TYPE_NAME(DgRosMappingUnsignedInt) = DgRosTypes::get_unsigned();
DG_DEFAULT_VALUE(DgRosMappingUnsignedInt) = 0.0;
DG_TO_ROS_IMPL(DgRosMappingUnsignedInt)
{
    ros_dst.data = dg_src;
}
ROS_TO_DG_IMPL(DgRosMappingUnsignedInt)
{
    dg_dst = ros_src.data;
}

/*
 * Matrix, DG and Ros matrix storage orders are row major.
 */

typedef DgRosMapping<RosMatrix, DgMatrix> DgRosMappingMatrix;
DG_SIGNAL_TYPE_NAME(DgRosMappingMatrix) = DgRosTypes::get_matrix();
DG_DEFAULT_VALUE(DgRosMappingMatrix) = DgMatrix::Zero(0, 0);
DG_TO_ROS_IMPL(DgRosMappingMatrix)
{
    // For simplicity and easiness of maintenance we implement a slower version
    // of the copy here.
    ros_dst.width = static_cast<unsigned int>(dg_src.rows());
    ros_dst.data.resize(dg_src.size());
    for (int row = 0; row < dg_src.rows(); ++row)
    {
        for (int col = 0; col < dg_src.cols(); ++col)
        {
            ros_dst.data[row * ros_dst.width + col] = dg_src(row, col);
        }
    }
}
ROS_TO_DG_IMPL(DgRosMappingMatrix)
{
    // For simplicity and easiness of maintenance we implement a slower version
    // of the copy here.
    int rows = ros_src.width;
    int cols = static_cast<int>(ros_src.data.size()) / static_cast<int>(rows);
    dg_dst.resize(rows, cols);
    for (int row = 0; row < dg_dst.rows(); ++row)
    {
        for (int col = 0; col < dg_dst.cols(); ++col)
        {
            dg_dst(row, col) = ros_src.data[row * rows + col];
        }
    }
}

/*
 * Vector
 */
typedef DgRosMapping<RosVector, DgVector> DgRosMappingVector;
DG_SIGNAL_TYPE_NAME(DgRosMappingVector) = DgRosTypes::get_vector();
DG_DEFAULT_VALUE(DgRosMappingVector) = DgVector::Zero(0);
DG_TO_ROS_IMPL(DgRosMappingVector)
{
    ros_dst.data.resize(static_cast<unsigned int>(dg_src.size()));
    for (unsigned int i = 0; i < dg_src.size(); ++i)
    {
        ros_dst.data[i] = dg_src[i];
    }
}
ROS_TO_DG_IMPL(DgRosMappingVector)
{
    dg_dst.resize(static_cast<int>(ros_src.data.size()));
    for (unsigned int i = 0; i < dg_dst.size(); ++i)
    {
        dg_dst[i] = ros_src.data[i];
    }
}

/*
 * Vector3, ROS is {x, y, z}, DG is a DgVector
 */
typedef DgRosMapping<geometry_msgs::msg::Vector3, DgVector> DgRosMappingVector3;
DG_SIGNAL_TYPE_NAME(DgRosMappingVector3) = DgRosTypes::get_vector3();
DG_DEFAULT_VALUE(DgRosMappingVector3) = DgVector::Zero(3);
DG_TO_ROS_IMPL(DgRosMappingVector3)
{
    if (dg_src.size() > 0)
    {
        ros_dst.x = dg_src(0);
        if (dg_src.size() > 1)
        {
            ros_dst.y = dg_src(1);
            if (dg_src.size() > 2)
            {
                ros_dst.z = dg_src(2);
            }
        }
    }
}
ROS_TO_DG_IMPL(DgRosMappingVector3)
{
    dg_dst.resize(3);
    dg_dst[0] = ros_src.x;
    dg_dst[1] = ros_src.y;
    dg_dst[2] = ros_src.z;
}

/*
 * MatrixHomo, ROS is a Transform (x, y, z, qx, qy, qz, qw),
 * DG is a homogeneous matrix.
 */
typedef DgRosMapping<geometry_msgs::msg::Transform, MatrixHomogeneous>
    DgRosMappingMatrixHomo;
DG_SIGNAL_TYPE_NAME(DgRosMappingMatrixHomo) = DgRosTypes::get_matrix_homogeneous();
DG_DEFAULT_VALUE(DgRosMappingMatrixHomo) = MatrixHomogeneous::Identity();
DG_TO_ROS_IMPL(DgRosMappingMatrixHomo)
{
    ros_dst.translation.x = dg_src.translation()(0);
    ros_dst.translation.y = dg_src.translation()(1);
    ros_dst.translation.z = dg_src.translation()(2);

    VectorQuaternion q(dg_src.linear());
    ros_dst.rotation.x = q.x();
    ros_dst.rotation.y = q.y();
    ros_dst.rotation.z = q.z();
    ros_dst.rotation.w = q.w();
}
ROS_TO_DG_IMPL(DgRosMappingMatrixHomo)
{
    dg_dst.translation()(0) = ros_src.translation.x;
    dg_dst.translation()(1) = ros_src.translation.y;
    dg_dst.translation()(2) = ros_src.translation.z;

    VectorQuaternion q(ros_src.rotation.w,
                       ros_src.rotation.x,
                       ros_src.rotation.y,
                       ros_src.rotation.z);
    dg_dst.linear() = q.matrix();
}

/*
 * MatrixHomo, ROS is a twist (x, y, z, wx, wy, wz),
 * DG is a homogeneous matrix.
 */
typedef DgRosMapping<geometry_msgs::msg::Twist, DgVector> DgRosMappingTwist;
DG_SIGNAL_TYPE_NAME(DgRosMappingTwist) = DgRosTypes::get_twist();
DG_DEFAULT_VALUE(DgRosMappingTwist) = DgVector::Zero(6);
DG_TO_ROS_IMPL(DgRosMappingTwist)
{
    if (dg_src.size() != 6)
    {
        throw std::runtime_error("failed to convert invalid twist");
    }
    ros_dst.linear.x = dg_src(0);
    ros_dst.linear.y = dg_src(1);
    ros_dst.linear.z = dg_src(2);
    ros_dst.angular.x = dg_src(3);
    ros_dst.angular.y = dg_src(4);
    ros_dst.angular.z = dg_src(5);
}
ROS_TO_DG_IMPL(DgRosMappingTwist)
{
    dg_dst.resize(6);
    dg_dst(0) = ros_src.linear.x;
    dg_dst(1) = ros_src.linear.y;
    dg_dst(2) = ros_src.linear.z;
    dg_dst(3) = ros_src.angular.x;
    dg_dst(4) = ros_src.angular.y;
    dg_dst(5) = ros_src.angular.z;
}

/*
 * Vector3Stamped, ROS is {x, y, z} stamped, DG is a DgVector
 */
typedef DgRosMapping<geometry_msgs::msg::Vector3Stamped, DgVector>
    DgRosMappingVector3Stamped;
DG_SIGNAL_TYPE_NAME(DgRosMappingVector3Stamped) = DgRosTypes::get_vector3_stamped();
DG_DEFAULT_VALUE(DgRosMappingVector3Stamped) = DgVector::Zero(3);
DG_TO_ROS_IMPL(DgRosMappingVector3Stamped)
{
    make_header(ros_dst.header);
    DgRosMappingVector3::dg_to_ros(dg_src, ros_dst.vector);
}
ROS_TO_DG_IMPL(DgRosMappingVector3Stamped)
{
    DgRosMappingVector3::ros_to_dg(ros_src.vector, dg_dst);
}

/*
 * MatrixHomo, ROS is a stamped Transform (x, y, z, qx, qy, qz, qw),
 * DG is a homogeneous matrix.
 */
typedef DgRosMapping<geometry_msgs::msg::TransformStamped, MatrixHomogeneous>
    DgRosMappingMatrixHomoStamped;
DG_SIGNAL_TYPE_NAME(DgRosMappingMatrixHomoStamped) =
    DgRosTypes::get_matrix_homogeneous_stamped();
DG_DEFAULT_VALUE(DgRosMappingMatrixHomoStamped) = MatrixHomogeneous::Identity();
DG_TO_ROS_IMPL(DgRosMappingMatrixHomoStamped)
{
    make_header(ros_dst.header);
    DgRosMappingMatrixHomo::dg_to_ros(dg_src, ros_dst.transform);
}
ROS_TO_DG_IMPL(DgRosMappingMatrixHomoStamped)
{
    DgRosMappingMatrixHomo::ros_to_dg(ros_src.transform, dg_dst);
}

/*
 * MatrixHomo, ROS is a stamped twist (x, y, z, wx, wy, wz),
 * DG is a homogeneous matrix.
 */
typedef DgRosMapping<geometry_msgs::msg::TwistStamped, DgVector>
    DgRosMappingTwistStamped;
DG_SIGNAL_TYPE_NAME(DgRosMappingTwistStamped) = DgRosTypes::get_twist_stamped();
DG_DEFAULT_VALUE(DgRosMappingTwistStamped) = DgVector::Zero(6);
DG_TO_ROS_IMPL(DgRosMappingTwistStamped)
{
    make_header(ros_dst.header);
    DgRosMappingTwist::dg_to_ros(dg_src, ros_dst.twist);
}
ROS_TO_DG_IMPL(DgRosMappingTwistStamped)
{
    DgRosMappingTwist::ros_to_dg(ros_src.twist, dg_dst);
}

}  // namespace dynamic_graph_manager
