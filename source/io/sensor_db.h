#pragma once
//
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
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
    float time    = 0.0f;
    float acc_x   = 0.0f;
    float acc_y   = 0.0f;
    float acc_z   = 0.0f;
    float gyro_x  = 0.0f;
    float gyro_y  = 0.0f;
    float gyro_z  = 0.0f;
    float mag_x   = 0.0f;
    float mag_y   = 0.0f;
    float mag_z   = 0.0f;
    float quate_x = 0.0f;
    float quate_y = 0.0f;
    float quate_z = 0.0f;
    float quate_w = 0.0f;
    float roll    = 0.0f;
    float pitch   = 0.0f;
    float yaw     = 0.0f;
    float eacc_x  = 0.0f;
    float eacc_y  = 0.0f;
    float eacc_z  = 0.0f;
    float vel_x   = 0.0f;
    float vel_y   = 0.0f;
    float vel_z   = 0.0f;
    float pos_x   = 0.0f;
    float pos_y   = 0.0f;
    float pos_z   = 0.0f;
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