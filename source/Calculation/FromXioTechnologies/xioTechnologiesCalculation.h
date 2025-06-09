
//
#pragma once
//
#include "Calculation/comput.h"
#include "Filtering/XioTechnologiesFusion/Fusion.h"
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
class xioTechnologiesCalculation
{
public:
    xioTechnologiesCalculation();
    ~xioTechnologiesCalculation(){};
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
    //
    // FusionMatrix gyroscopeMisalignment = {
    //     1.0f,        0.000735988f, 0.0404814f,    //
    //     -0.0050869f, 1.0f,         -0.00886337f,  //
    //     -0.0131379f, 0.00531571f,  1.0f           //
    // };
    // FusionVector gyroscopeSensitivity = { 1.000133557f, 1.000131974f, 1.000132885f };
    // FusionVector gyroscopeOffset      = { -0.00417839816801879115965502212835f, 0.00117866481018117214717140691168f, -0.00679347428184092095880127363773f };
    // //
    // FusionMatrix accelerometerMisalignment = {
    //     1.0f, 0.00374111f, 0.0356312f,    //
    //     0.0f, 1.0f,        -0.00972883f,  //
    //     0.0f, 0.0f,        1.0f           //
    // };
    // FusionVector accelerometerSensitivity = { 1.000598645f, 1.000599214f, 1.00059699f };
    // FusionVector accelerometerOffset      = { -0.005330224609375f, -0.00171549072265625f, 0.023236083984375f };
    // //
    // FusionMatrix softIronMatrix = {
    //     1.0f, 0.0f, 0.0f,  //
    //     0.0f, 1.0f, 0.0f,  //
    //     0.0f, 0.0f, 1.0f   //
    // };
    // FusionVector hardIronOffset = { 0.0f, 0.0f, 0.0f };
public:
    int64_t start_time = 0;
    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs   ahrs;
    int64_t      previousTimestamp;
    bool         one_previousAcceleration_init = false;
    bool         mul_previousAcceleration_init = false;
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
    bool Mul_SolveAnCalculation( EIGEN_SENSOR_DATA* sensor_data, EIGEN_SENSOR_DATA* original_sensor_data );
    bool Mul_CalculateVelAndPos( EIGEN_SENSOR_DATA* sensor_data, float dt, bool is_hp );
    //
    bool One_SolveAnCalculation( EIGEN_SENSOR_DATA* sensor_data );
    bool One_CalculateVelAndPos( EIGEN_SENSOR_DATA* sensor_data, float dt, bool is_hp );
    //
    void        ResetInitial();
    void        ResetInitFusion();
    void        ConfigFusion( std::string content );
    std::string GetConfigString();
    void        read_config();
private:
    //
    void save_config();
};
