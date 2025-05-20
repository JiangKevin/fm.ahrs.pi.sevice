
//
#pragma once
//
#include "Filtering/XioTechnologiesFusion/Fusion.h"
#include "Calculation/comput.h"
#include "concurrentqueue/concurrentqueue.h"
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
    FusionVector initialVelocity          = { 0.0f, 0.0f, 0.0f };
    FusionVector original_initialVelocity = { 0.0f, 0.0f, 0.0f };

    // 初始位置
    FusionVector initialPosition          = { 0.0f, 0.0f, 0.0f };
    FusionVector original_initialPosition = { 0.0f, 0.0f, 0.0f };
public:
    int64_t start_time = 0;
    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs   ahrs;
    int64_t      previousTimestamp;
    bool         previousAcceleration_init = false;
    float        deltaTime;
    // Set AHRS algorithm settings
    FusionAhrsSettings settings = {
        .convention            = FusionConventionEnu,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    //
public:
    bool        SolveAnCalculation( SENSOR_DB* sensor_data, SENSOR_DB* original_sensor_data );
    bool        CalculateVelAndPos( SENSOR_DB* sensor_data, float dt, bool is_gd );
    void        ResetInitial();
    void        ResetInitFusion();
    void        ConfigFusion( std::string content );
    std::string GetConfigString();
private:
};
