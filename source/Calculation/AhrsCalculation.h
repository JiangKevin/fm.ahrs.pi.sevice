
//
#pragma once
//
#include "Fusion/Fusion.h"
#include "concurrentqueue/concurrentqueue.h"
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

// 以纳秒级精度获取当前时间戳
static long long getNanosecondTimestamp()
{
    auto now      = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast< std::chrono::nanoseconds >( duration ).count();
}
// 以微秒级精度获取当前时间戳
static int64_t getMicrosecondTimestamp()
{
    auto now      = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast< std::chrono::microseconds >( duration ).count();
}
//
static std::vector< std::string > splitString( const std::string& str, char delimiter )
{
    std::vector< std::string > result;
    std::istringstream         iss( str );
    std::string                token;
    while ( std::getline( iss, token, delimiter ) )
    {
        result.push_back( token );
    }
    return result;
}
//
static std::string transaction_to_string( float value )
{
    std::string out;
    if ( std::isnan( value ) )
    {
        out = "0";
    }
    else
    {
        out = std::to_string( value );
    }
    //
    return out;
}
//
struct SENSOR_DB
{
    int64_t time;
    float   acc_x  = 0.0f;
    float   acc_y  = 0.0f;
    float   acc_z  = 0.0f;
    float   gyro_x = 0.0f;
    float   gyro_y = 0.0f;
    float   gyro_z = 0.0f;
    float   mag_x  = 0.0f;
    float   mag_y  = 0.0f;
    float   mag_z  = 0.0f;

    float quate_x = 0.0f;
    float quate_y = 0.0f;
    float quate_z = 0.0f;
    float quate_w = 0.0f;
    float roll    = 0.0f;
    float pitch   = 0.0f;
    float yaw     = 0.0f;

    float eacc_x = 0.0f;
    float eacc_y = 0.0f;
    float eacc_z = 0.0f;

    float vel_x = 0.0f;
    float vel_y = 0.0f;
    float vel_z = 0.0f;
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float pos_z = 0.0f;
    //
    std::string to_string()
    {
        std::string str = transaction_to_string( time ) + ",";
        str += transaction_to_string( acc_x ) + "," + transaction_to_string( acc_y ) + "," + transaction_to_string( acc_z ) + ",";
        str += transaction_to_string( gyro_x ) + "," + transaction_to_string( gyro_y ) + "," + transaction_to_string( gyro_z ) + ",";
        str += transaction_to_string( mag_x ) + "," + transaction_to_string( mag_y ) + "," + transaction_to_string( mag_z ) + ",";
        str += transaction_to_string( quate_x ) + "," + transaction_to_string( quate_y ) + "," + transaction_to_string( quate_z ) + "," + transaction_to_string( quate_w ) + ",";
        str += transaction_to_string( roll ) + "," + transaction_to_string( pitch ) + "," + transaction_to_string( yaw ) + ",";
        str += transaction_to_string( eacc_x ) + "," + transaction_to_string( eacc_y ) + "," + transaction_to_string( eacc_z ) + ",";
        str += transaction_to_string( vel_x ) + "," + transaction_to_string( vel_y ) + "," + transaction_to_string( vel_z ) + ",";
        str += transaction_to_string( pos_x ) + "," + transaction_to_string( pos_y ) + "," + transaction_to_string( pos_z );
        return str;
    };
    //
    std::string to_info()
    {
        std::string info = "Time: " + transaction_to_string( time ) + "\n";
        info += "Accelerometer: (" + transaction_to_string( acc_x ) + ", " + transaction_to_string( acc_y ) + ", " + transaction_to_string( acc_z ) + ")\n";
        info += "Gyroscope: (" + transaction_to_string( gyro_x ) + ", " + transaction_to_string( gyro_y ) + ", " + transaction_to_string( gyro_z ) + ")\n";
        info += "Magnetometer: (" + transaction_to_string( mag_x ) + ", " + transaction_to_string( mag_y ) + ", " + transaction_to_string( mag_z ) + ")\n";
        info += "Quaternion: (" + transaction_to_string( quate_x ) + ", " + transaction_to_string( quate_y ) + ", " + transaction_to_string( quate_z ) + ", " + transaction_to_string( quate_w ) + ")\n";
        info += "Roll: " + transaction_to_string( roll ) + " pitch: " + transaction_to_string( pitch ) + " yaw: " + transaction_to_string( yaw ) + "\n";
        info += "Estimated Accelerometer: (" + transaction_to_string( eacc_x ) + ", " + transaction_to_string( eacc_y ) + ", " + transaction_to_string( eacc_z ) + ")\n";
        info += "Estimated Velocity: (" + transaction_to_string( vel_x ) + ", " + transaction_to_string( vel_y ) + ", " + transaction_to_string( vel_z ) + ")\n";
        info += "Position: (" + transaction_to_string( pos_x ) + ", " + transaction_to_string( pos_y ) + ", " + transaction_to_string( pos_z ) + ")\n";
        return info;
    }
    //
    void getValueFromString( std::string v )
    {
        char delimiter = ',';
        auto values    = splitString( v, delimiter );
        //
        if ( values.size() == 26 )
        {
            time    = std::stof( values[ 0 ] );
            acc_x   = std::stof( values[ 1 ] );
            acc_y   = std::stof( values[ 2 ] );
            acc_z   = std::stof( values[ 3 ] );
            gyro_x  = std::stof( values[ 4 ] );
            gyro_y  = std::stof( values[ 5 ] );
            gyro_z  = std::stof( values[ 6 ] );
            mag_x   = std::stof( values[ 7 ] );
            mag_y   = std::stof( values[ 8 ] );
            mag_z   = std::stof( values[ 9 ] );
            quate_x = std::stof( values[ 10 ] );
            quate_y = std::stof( values[ 11 ] );
            quate_z = std::stof( values[ 12 ] );
            quate_w = std::stof( values[ 13 ] );
            roll    = std::stof( values[ 14 ] );
            pitch   = std::stof( values[ 15 ] );
            yaw     = std::stof( values[ 16 ] );
            eacc_x  = std::stof( values[ 17 ] );
            eacc_y  = std::stof( values[ 18 ] );
            eacc_z  = std::stof( values[ 19 ] );
            vel_x   = std::stof( values[ 20 ] );
            vel_y   = std::stof( values[ 21 ] );
            vel_z   = std::stof( values[ 22 ] );
            pos_x   = std::stof( values[ 23 ] );
            pos_y   = std::stof( values[ 24 ] );
            pos_z   = std::stof( values[ 25 ] );
        }
    }
};
// 从加速度计算位移的函数
struct MotionData
{
    std::vector< float > time;
    std::vector< float > acceleration;
    std::vector< float > velocity;
    std::vector< float > displacement;
};
// 验证字符串是否为数字
static bool isNumber( const std::string& str )
{
    for ( char c : str )
    {
        if ( ! std::isdigit( c ) )
        {
            return false;
        }
    }
    return true;
}
//
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
    FusionVector accelerometerOffset       = { 0.0f, 0.0f, 0.025f };
    //
    FusionMatrix softIronMatrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    FusionVector hardIronOffset = { 0.0f, 0.0f, 0.0f };
    // 初始速度
    FusionVector initialVelocity = { 0.0f, 0.0f, 0.0f };
    // 初始位置
    FusionVector initialPosition = { 0.0f, 0.0f, 0.0f };
public:
    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs   ahrs;
    int64_t      previousTimestamp;
    float        deltaTime;
    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
        .convention            = FusionConventionNwu,
        .gain                  = 0.5f,
        .gyroscopeRange        = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
        .accelerationRejection = 10.0f,
        .magneticRejection     = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
public:
    void SolveAnCalculation( SENSOR_DB* sensor_data );
    void ResetInitial();
private:
    void calculateSurfaceVelocity( SENSOR_DB* sensor_data, float dt );
};
