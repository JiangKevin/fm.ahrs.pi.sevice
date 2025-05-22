#pragma once
//
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <vector>
//
static std::string removePrefix( const std::string& str, const std::string& prefix )
{
    if ( str.find( prefix ) == 0 )
    {
        return str.substr( prefix.length() );
    }
    return str;
}
//
static bool startsWith( const std::string& str, const std::string& prefix )
{
    return str.size() >= prefix.size() && str.compare( 0, prefix.size(), prefix ) == 0;
}
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
static std::string int_transaction_to_string( int value )
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
static std::string int64_transaction_to_string( int64_t value )
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
struct EIGEN_SENSOR_DATA
{
    int64_t         time      = 0;
    Eigen::VectorXf acc       = Eigen::VectorXf::Zero( 3 );
    Eigen::VectorXf gyr       = Eigen::VectorXf::Zero( 3 );
    Eigen::VectorXf mag       = Eigen::VectorXf::Zero( 3 );
    Eigen::VectorXf qua       = Eigen::VectorXf::Zero( 4 );
    Eigen::VectorXf eul       = Eigen::VectorXf::Zero( 3 );
    Eigen::VectorXf eacc      = Eigen::VectorXf::Zero( 3 );
    Eigen::VectorXf vel       = Eigen::VectorXf::Zero( 3 );
    Eigen::VectorXf pos       = Eigen::VectorXf::Zero( 3 );
    float           deltaTime = 0.0f;

    //
    std::string to_string()
    {
        std::string str = int64_transaction_to_string( time ) + ",";
        str += transaction_to_string( acc[ 0 ] ) + "," + transaction_to_string( acc[ 1 ] ) + "," + transaction_to_string( acc[ 2 ] ) + ",";
        str += transaction_to_string( gyr[ 0 ] ) + "," + transaction_to_string( gyr[ 1 ] ) + "," + transaction_to_string( gyr[ 2 ] ) + ",";
        str += transaction_to_string( mag[ 0 ] ) + "," + transaction_to_string( mag[ 1 ] ) + "," + transaction_to_string( mag[ 2 ] ) + ",";
        str += transaction_to_string( qua[ 0 ] ) + "," + transaction_to_string( qua[ 1 ] ) + "," + transaction_to_string( qua[ 2 ] ) + "," + transaction_to_string( qua[ 3 ] ) + ",";
        str += transaction_to_string( eul[ 0 ] ) + "," + transaction_to_string( eul[ 0 ] ) + "," + transaction_to_string( eul[ 2 ] ) + ",";
        str += transaction_to_string( eacc[ 0 ] ) + "," + transaction_to_string( eacc[ 1 ] ) + "," + transaction_to_string( eacc[ 2 ] ) + ",";
        str += transaction_to_string( vel[ 0 ] ) + "," + transaction_to_string( vel[ 1 ] ) + "," + transaction_to_string( vel[ 2 ] ) + ",";
        str += transaction_to_string( pos[ 0 ] ) + "," + transaction_to_string( pos[ 1 ] ) + "," + transaction_to_string( pos[ 2 ] ) + ",";
        str += transaction_to_string( deltaTime );
        //
        return str;
    }
    //
    std::string to_info()
    {
        std::string info = "Time: " + int64_transaction_to_string( time ) + "\n";
        info += "Accelerometer: (" + transaction_to_string( acc[ 0 ] ) + ", " + transaction_to_string( acc[ 1 ] ) + ", " + transaction_to_string( acc[ 2 ] ) + ")\n";
        info += "Gyroscope: (" + transaction_to_string( gyr[ 0 ] ) + ", " + transaction_to_string( gyr[ 1 ] ) + ", " + transaction_to_string( gyr[ 2 ] ) + ")\n";
        info += "Magnetometer: (" + transaction_to_string( mag[ 0 ] ) + ", " + transaction_to_string( mag[ 1 ] ) + ", " + transaction_to_string( mag[ 2 ] ) + ")\n";
        info += "Quaternion: (" + transaction_to_string( qua[ 0 ] ) + ", " + transaction_to_string( qua[ 1 ] ) + ", " + transaction_to_string( qua[ 2 ] ) + ", " + transaction_to_string( qua[ 3 ] ) + ")\n";
        info += "Roll: " + transaction_to_string( eul[ 0 ] ) + " pitch: " + transaction_to_string( eul[ 1 ] ) + " yaw: " + transaction_to_string( eul[ 2 ] ) + "\n";
        info += "Estimated Accelerometer: (" + transaction_to_string( eacc[ 0 ] ) + ", " + transaction_to_string( eacc[ 1 ] ) + ", " + transaction_to_string( eacc[ 2 ] ) + ")\n";
        info += "Estimated Velocity: (" + transaction_to_string( vel[ 0 ] ) + ", " + transaction_to_string( vel[ 1 ] ) + ", " + transaction_to_string( vel[ 2 ] ) + ")\n";
        info += "Position: (" + transaction_to_string( pos[ 0 ] ) + ", " + transaction_to_string( pos[ 1 ] ) + ", " + transaction_to_string( pos[ 2 ] ) + ")\n";
        info += "deltaTime: (" + transaction_to_string( deltaTime ) + ")\n";

        return info;
    }
    //
    void getValueFromString( std::string v )
    {
        char delimiter = ',';
        auto values    = splitString( v, delimiter );
        //
        if ( values.size() == 27 )
        {
            time      = std::stoll( values[ 0 ] );
            acc[ 0 ]  = std::stof( values[ 1 ] );
            acc[ 1 ]  = std::stof( values[ 2 ] );
            acc[ 2 ]  = std::stof( values[ 3 ] );
            gyr[ 0 ]  = std::stof( values[ 4 ] );
            gyr[ 1 ]  = std::stof( values[ 5 ] );
            gyr[ 2 ]  = std::stof( values[ 6 ] );
            mag[ 0 ]  = std::stof( values[ 7 ] );
            mag[ 1 ]  = std::stof( values[ 8 ] );
            mag[ 2 ]  = std::stof( values[ 9 ] );
            qua[ 0 ]  = std::stof( values[ 10 ] );
            qua[ 1 ]  = std::stof( values[ 11 ] );
            qua[ 2 ]  = std::stof( values[ 12 ] );
            qua[ 3 ]  = std::stof( values[ 13 ] );
            eul[ 0 ]  = std::stof( values[ 14 ] );
            eul[ 1 ]  = std::stof( values[ 15 ] );
            eul[ 2 ]  = std::stof( values[ 16 ] );
            eacc[ 0 ] = std::stof( values[ 17 ] );
            eacc[ 1 ] = std::stof( values[ 18 ] );
            eacc[ 2 ] = std::stof( values[ 19 ] );
            vel[ 0 ]  = std::stof( values[ 20 ] );
            vel[ 1 ]  = std::stof( values[ 21 ] );
            vel[ 2 ]  = std::stof( values[ 22 ] );
            pos[ 0 ]  = std::stof( values[ 23 ] );
            pos[ 1 ]  = std::stof( values[ 24 ] );
            pos[ 2 ]  = std::stof( values[ 25 ] );
            deltaTime = std::stof( values[ 26 ] );
        }
    }
    //
    void ToZero()
    {
        time      = 0.0f;
        acc[ 0 ]  = 0.0f;
        acc[ 1 ]  = 0.0f;
        acc[ 2 ]  = 0.0f;
        gyr[ 0 ]  = 0.0f;
        gyr[ 1 ]  = 0.0f;
        gyr[ 2 ]  = 0.0f;
        mag[ 0 ]  = 0.0f;
        mag[ 1 ]  = 0.0f;
        mag[ 2 ]  = 0.0f;
        qua[ 0 ]  = 0.0f;
        qua[ 1 ]  = 0.0f;
        qua[ 2 ]  = 0.0f;
        qua[ 3 ]  = 0.0f;
        eul[ 0 ]  = 0.0f;
        eul[ 1 ]  = 0.0f;
        eul[ 2 ]  = 0.0f;
        eacc[ 0 ] = 0.0f;
        eacc[ 1 ] = 0.0f;
        eacc[ 2 ] = 0.0f;
        vel[ 0 ]  = 0.0f;
        vel[ 1 ]  = 0.0f;
        vel[ 2 ]  = 0.0f;
        pos[ 0 ]  = 0.0f;
        pos[ 1 ]  = 0.0f;
        pos[ 2 ]  = 0.0f;
        deltaTime = 0.0f;
    }
};