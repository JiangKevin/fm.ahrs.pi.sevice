
//
#pragma once
//
#include "Filtering/LowPass/LowPassFilter.h"
#include "Filtering/kalman/ExtendedAccelerationKalmanFilter.h"
#include "Fusion/Fusion.h"
#include "concurrentqueue/concurrentqueue.h"
#include "sensor_db.h"
#include <Eigen/Dense>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <vector>

//
#define SAMPLE_RATE ( 100 )  // replace this with actual sample rate

// 从加速度计算位移的函数
struct MotionData
{
    std::vector< float > time;
    std::vector< float > acceleration;
    std::vector< float > velocity;
    std::vector< float > displacement;
};
//
class AhrsCalculation
{
public:
    AhrsCalculation();
    ~AhrsCalculation(){};
public:
    // Define calibration (replace with actual calibration data if available)
    FusionMatrix gyroscopeMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector gyroscopeSensitivity  = { 1.0f, 1.0f, 1.0f };
    FusionVector gyroscopeOffset       = { 0.0f, 0.0f, 0.0f };
    //
    FusionMatrix accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector accelerometerSensitivity  = { 1.0f, 1.0f, 1.0f };
    FusionVector accelerometerOffset       = { 0.001f, 0.007f, 0.031f };
    //
    FusionMatrix softIronMatrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector hardIronOffset = { 0.0f, 0.0f, 0.0f };
    // 初始速度
    FusionVector initialVelocity = { 0.0f, 0.0f, 0.0f };
    // 初始位置
    FusionVector initialPosition = { 0.0f, 0.0f, 0.0f };
public:
    int64_t start_time = 0;
    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs   ahrs;
    int64_t      previousTimestamp;
    FusionVector previousAcceleration      = { 0.0f, 0.0f, 0.0f };
    bool         previousAcceleration_init = false;
    float        deltaTime;
    // Set AHRS algorithm settings
    FusionAhrsSettings settings = {
        .convention            = FusionConventionNwu,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    //
    // 创建卡尔曼滤波对象，使用空构造函数
    ExtendedAccelerationKalmanFilter akf;
public:
    bool        SolveAnCalculation( SENSOR_DB* sensor_data, SENSOR_DB* original_sensor_data );
    void        ResetInitial();
    void        ResetInitFusion();
    void        ConfigFusion( std::string content );
    std::string GetConfigString();
private:
    bool calculateSurfaceVelocity( SENSOR_DB* sensor_data, float dt, bool is_lp );
    //
    // 修改后的函数：采用低通滤波器对三个轴的加速度进行滤波，
    // 当经过滤波后的每个轴如果低于各自的阈值时，认为该轴输出 0；
    // 函数接口增加三个阈值参数：threshold_x, threshold_y, threshold_z
    void filterAcceleration( float ax, float ay, float az, float& out_ax, float& out_ay, float& out_az, float threshold_x, float threshold_y, float threshold_z );
    //
    void UseKF( SENSOR_DB* sensor_data, float dt );
    void initKF();
};
