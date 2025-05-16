
#pragma once
// /
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

// 结构体定义传感器数据
struct SensorData
{
    Eigen::Vector3f gyro;   // 陀螺仪数据
    Eigen::Vector3f accel;  // 加速度计数据
    Eigen::Vector3f mag;    // 磁力计数据
};

// 计算旋转矩阵并转换为欧拉角
static Eigen::Vector3f computeEulerAngles( const SensorData& data )
{
    Eigen::Vector3f accelNorm = data.accel.normalized();
    Eigen::Vector3f magNorm   = data.mag.normalized();

    // 计算旋转矩阵
    Eigen::Vector3f west  = -accelNorm.cross( magNorm ).normalized();
    Eigen::Vector3f north = west.cross( accelNorm ).normalized();

    Eigen::Matrix3f rotMatrix;
    rotMatrix.col( 0 ) = west;
    rotMatrix.col( 1 ) = north;
    rotMatrix.col( 2 ) = accelNorm;

    // 计算欧拉角（Yaw, Pitch, Roll）
    Eigen::Vector3f eulerAngles = rotMatrix.eulerAngles( 2, 1, 0 );  // Yaw, Pitch, Roll
    return eulerAngles;
}

// 计算线性加速度（去除重力影响）
static Eigen::Vector3f computeLinearAcceleration( const SensorData& data, const Eigen::Vector3f& eulerAngles )
{
    Eigen::Quaternionf quat = Eigen::AngleAxisf( eulerAngles[ 0 ], Eigen::Vector3f::UnitZ() ) * Eigen::AngleAxisf( eulerAngles[ 1 ], Eigen::Vector3f::UnitY() ) * Eigen::AngleAxisf( eulerAngles[ 2 ], Eigen::Vector3f::UnitX() );

    Eigen::Vector3f gravity = quat * Eigen::Vector3f( 0, 0, 9.81f );
    return data.accel - gravity;
}
//
static Eigen::VectorXf computeVelocityOfTrapezoid( float dt, const Eigen::VectorXf& a_prev, const Eigen::VectorXf& a_next )
{
    Eigen::VectorXf v = 0.5 * ( a_prev + a_next ) * dt;
    return v;
}
