#pragma once
//

#include <Eigen/Dense>
#include <stdexcept>

// 旋转顺序枚举
enum class RotationOrder
{
    ZYX,  // Yaw-Pitch-Roll (Tait-Bryan angles)
    XYZ,  // Roll-Pitch-Yaw
    ZXY,  // Yaw-Roll-Pitch
    XZY,  // Roll-Yaw-Pitch
    YZX,  // Pitch-Yaw-Roll
    YXZ   // Pitch-Roll-Yaw
};

/**
 * 通过欧拉角旋转向量
 * @param vector 待旋转的向量
 * @param roll 绕X轴旋转角度（弧度）
 * @param pitch 绕Y轴旋转角度（弧度）
 * @param yaw 绕Z轴旋转角度（弧度）
 * @param order 旋转顺序，默认为ZYX (Yaw-Pitch-Roll)
 * @return 旋转后的向量
 */
static Eigen::Vector3f fm_rotateVector( const Eigen::Vector3f& vector, float roll, float pitch, float yaw, RotationOrder order = RotationOrder::XYZ )
{

    Eigen::Matrix3f rotationMatrix;

    switch ( order )
    {
        case RotationOrder::ZYX:
            rotationMatrix = Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() ) * Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() ) * Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() );
            break;

        case RotationOrder::XYZ:
            rotationMatrix = Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() ) * Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() ) * Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() );
            break;

        case RotationOrder::ZXY:
            rotationMatrix = Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() ) * Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() ) * Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() );
            break;

        case RotationOrder::XZY:
            rotationMatrix = Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() ) * Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() ) * Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() );
            break;

        case RotationOrder::YZX:
            rotationMatrix = Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() ) * Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() ) * Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() );
            break;

        case RotationOrder::YXZ:
            rotationMatrix = Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() ) * Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() ) * Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() );
            break;

        default:
            throw std::invalid_argument( "Unsupported rotation order" );
    }

    return rotationMatrix * vector;
}

