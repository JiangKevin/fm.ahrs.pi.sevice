
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
static int              start_dalta_index    = 0;
// static int              dalta_index          = 0;

//
static Eigen::VectorXf computeVelocityOfTrapezoid( float dt, const Eigen::VectorXf& a_prev, const Eigen::VectorXf& a_next )
{
    Eigen::VectorXf v = 0.5 * ( a_prev + a_next ) * dt;
    return v;
}
//
// 判断设备是否静止：对于 x, y, z 轴分别设置不同的阈值
// 参数 axesThreshold 的每个分量分别对应 x, y, z 轴的阈值
static bool isStationary( float& netAc_x, float& netAc_y, float& netAc_z, float axesThreshold_x, float axesThreshold_y, float axesThreshold_z )
{
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
    // 当所有轴均等于 0.0f 时，认为设备处于静止状态
    return ( netAc_x == 0.0f && netAc_y == 0.0f && netAc_z == 0.0f );
}
// 使用 Eigen::VectorXf 表示动态长度向量
// 这里假设传感器加速度数据为 3 维向量（即传感器的 x, y, z 分量）
static Eigen::VectorXf getLinearAcceleration( const Eigen::VectorXf sensorAcc, const Eigen::Quaternionf q )
{
    // sensorAcc 期望为3x1向量，内容为传感器坐标系下的加速度（单位：g）
    // 利用四元数将传感器加速度旋转到地球坐标系
    Eigen::VectorXf earthAcc = q * sensorAcc;

    // 去除重力分量：
    // 假定在此地球坐标系（例如 ENU 或 NWU）中，重力 1g 作用在 Z 轴正方向，
    // 因此将 Z 分量减去 1g 得到纯粹的线性加速度
    if ( earthAcc.size() >= 3 )
    {
        earthAcc( 2 ) -= 1.0f;
        //
        earthAcc[ 1 ] -= 0.39f;
        earthAcc[ 2 ] += 0.08f;
    }

    //
    return earthAcc;
}
//
static void getLinearAccFromSd( SENSOR_DB* sd )
{
    // 创建一个 3 维传感器加速度数据向量，单位假定为 g。
    // 例如：静止状态下（只受重力作用），传感器应测得 1g 的重力加速度在 Z 轴上
    Eigen::VectorXf sensorAcc( 3 );
    sensorAcc << sd->acc_x, sd->acc_y, sd->acc_z;

    // 定义一个四元数，此处使用单位四元数（无旋转），
    // 表示传感器坐标系与地球坐标系一致
    Eigen::Quaternionf q( sd->quate_x, sd->quate_y, sd->quate_z, sd->quate_w );
    //
    // 计算线性加速度（剔除了重力的加速度）
    Eigen::VectorXf linearAcc = getLinearAcceleration( sensorAcc, q );
    //
    sd->eacc_x = linearAcc[ 0 ];
    sd->eacc_y = linearAcc[ 1 ];
    sd->eacc_z = linearAcc[ 2 ];
}