
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
static Eigen::VectorXf one_previousAcceleration = Eigen::VectorXf::Zero( 3 );
static Eigen::VectorXf mul_previousAcceleration = Eigen::VectorXf::Zero( 3 );
static int             start_dalta_index        = 0;
//
static Eigen::VectorXf computeVelocityOfTrapezoid( float dt, const Eigen::VectorXf& a_prev, const Eigen::VectorXf& a_next )
{
    Eigen::VectorXf v = 0.5 * ( a_prev + a_next ) * 9.8 * dt;
    return v;
}
//
// 判断设备是否静止：对于 x, y, z 轴分别设置不同的阈值
// 参数 axesThreshold 的每个分量分别对应 x, y, z 轴的阈值
static bool isStationary( EIGEN_SENSOR_DATA* sensor_data, float axesThreshold_x, float axesThreshold_y, float axesThreshold_z )
{
    float netAc_x = sensor_data->eacc[ 0 ], netAc_y = sensor_data->eacc[ 1 ], netAc_z = sensor_data->eacc[ 2 ];
    //
    if ( std::abs( netAc_x ) < axesThreshold_x )
    {
        netAc_x = 0.0f;
    }
    if ( std::abs( netAc_y ) < axesThreshold_y )
    {
        netAc_y = 0.0f;
    }
    if ( std::abs( netAc_z ) < axesThreshold_z )
    {
        netAc_z = 0.0f;
    }
    //
    sensor_data->eacc[ 0 ] = netAc_x;
    sensor_data->eacc[ 1 ] = netAc_y;
    sensor_data->eacc[ 2 ] = netAc_z;
    // 当所有轴均等于 0.0f 时，认为设备处于静止状态
    return ( netAc_x == 0.0f && netAc_y == 0.0f && netAc_z == 0.0f );
}
