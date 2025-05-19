
#pragma once
// /
#include "OpenKF/kalman_filter.h"
#include "OpenKF/types.h"
#include "sensor_db.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
//
// 定义状态向量维度：3 维位置 + 3 维速度
static constexpr size_t DIM_X                = 6;
static Eigen::VectorXf  previousAcceleration = Eigen::VectorXf::Zero( 3 );
static int              dalta_index          = 0;
static int              start_dalta_index    = 0;
//
static Eigen::VectorXf computeVelocityOfTrapezoid( float dt, const Eigen::VectorXf& a_prev, const Eigen::VectorXf& a_next )
{
    Eigen::VectorXf v = 0.5 * ( a_prev + a_next ) * dt;
    return v;
}
//
// 判断设备是否静止：对于 x, y, z 轴分别设置不同的阈值
// 参数 axesThreshold 的每个分量分别对应 x, y, z 轴的阈值
static bool isStationary( Eigen::Vector3f& netAcc, const Eigen::Vector3f& axesThreshold )
{
    if ( std::abs( netAcc.x() ) < axesThreshold.x() )
    {
        netAcc.x() = 0.0f;
    }
    if ( std::abs( netAcc.y() ) < axesThreshold.y() )
    {
        netAcc.y() = 0.0f;
    }
    if ( std::abs( netAcc.z() ) < axesThreshold.z() )
    {
        netAcc.z() = 0.0f;
    }
    // 当所有轴均等于 0.0f 时，认为设备处于静止状态
    return ( netAcc.x() == 0.0f && netAcc.y() == 0.0f && netAcc.z() == 0.0f );
}
