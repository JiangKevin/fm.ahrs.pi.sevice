#pragma once
//
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
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
    float           totalAcc  = 0.0f;
    //
    std::string to_json_string()
    {
        nlohmann::json j;
        j[ "time" ]      = time;
        j[ "acc" ]       = { acc[ 0 ], acc[ 1 ], acc[ 2 ] };
        j[ "gyr" ]       = { gyr[ 0 ], gyr[ 1 ], gyr[ 2 ] };
        j[ "mag" ]       = { mag[ 0 ], mag[ 1 ], mag[ 2 ] };
        j[ "qua" ]       = { qua[ 0 ], qua[ 1 ], qua[ 2 ], qua[ 3 ] };
        j[ "eul" ]       = { eul[ 0 ], eul[ 1 ], eul[ 2 ] };
        j[ "eacc" ]      = { eacc[ 0 ], eacc[ 1 ], eacc[ 2 ] };
        j[ "vel" ]       = { vel[ 0 ], vel[ 1 ], vel[ 2 ] };
        j[ "pos" ]       = { pos[ 0 ], pos[ 1 ], pos[ 2 ] };
        j[ "deltaTime" ] = deltaTime;
        j[ "totalAcc" ]  = totalAcc;
        //
        return j.dump();
    }
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
        str += transaction_to_string( deltaTime ) + ",";
        str += transaction_to_string( totalAcc );

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
        info += "totalAcc: (" + transaction_to_string( totalAcc ) + ")\n";

        return info;
    }
    //
    void getValueFromJsonString( std::string v )
    {
        char delimiter = '|';
        auto values    = splitString( v, delimiter );
        //
        if ( values.size() == 2 )
        {
            std::stringstream ss( values[ 1 ] );
            // std::cout << "Json String: " << values[ 1 ] << std::endl;
            nlohmann::json j = nlohmann::json::parse( ss, nullptr, false );
            //
            time      = j[ "time" ].get< int64_t >();
            acc[ 0 ]  = j[ "acc" ][ 0 ].get< float >();
            acc[ 1 ]  = j[ "acc" ][ 1 ].get< float >();
            acc[ 2 ]  = j[ "acc" ][ 2 ].get< float >();
            gyr[ 0 ]  = j[ "gyr" ][ 0 ].get< float >();
            gyr[ 1 ]  = j[ "gyr" ][ 1 ].get< float >();
            gyr[ 2 ]  = j[ "gyr" ][ 2 ].get< float >();
            mag[ 0 ]  = j[ "mag" ][ 0 ].get< float >();
            mag[ 1 ]  = j[ "mag" ][ 1 ].get< float >();
            mag[ 2 ]  = j[ "mag" ][ 2 ].get< float >();
            qua[ 0 ]  = j[ "qua" ][ 0 ].get< float >();
            qua[ 1 ]  = j[ "qua" ][ 1 ].get< float >();
            qua[ 2 ]  = j[ "qua" ][ 2 ].get< float >();
            qua[ 3 ]  = j[ "qua" ][ 3 ].get< float >();
            eul[ 0 ]  = j[ "eul" ][ 0 ].get< float >();
            eul[ 1 ]  = j[ "eul" ][ 1 ].get< float >();
            eul[ 2 ]  = j[ "eul" ][ 2 ].get< float >();
            eacc[ 0 ] = j[ "eacc" ][ 0 ].get< float >();
            eacc[ 1 ] = j[ "eacc" ][ 1 ].get< float >();
            eacc[ 2 ] = j[ "eacc" ][ 2 ].get< float >();
            vel[ 0 ]  = j[ "vel" ][ 0 ].get< float >();
            vel[ 1 ]  = j[ "vel" ][ 1 ].get< float >();
            vel[ 2 ]  = j[ "vel" ][ 2 ].get< float >();
            pos[ 0 ]  = j[ "pos" ][ 0 ].get< float >();
            pos[ 1 ]  = j[ "pos" ][ 1 ].get< float >();
            pos[ 2 ]  = j[ "pos" ][ 2 ].get< float >();
            deltaTime = j[ "deltaTime" ].get< float >();
            totalAcc  = j[ "totalAcc" ].get< float >();
            //
        }
    }
    //
    void getValueFromString( std::string v )
    {
        char delimiter = ',';
        auto values    = splitString( v, delimiter );
        //
        if ( values.size() == 28 )
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
            totalAcc  = std::stof( values[ 27 ] );
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
        totalAcc  = 0.0f;
    }
    //
    void ToFusionZero()
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
        totalAcc  = 0.0f;
    }
};
//
struct MAG_FIELD_FINGERPRINT
{
    float  mag_x  = 0.0f;
    float  mag_y  = 0.0f;
    float  mag_z  = 0.0f;
    float  rela_x = 0.0f;  // 相对方向
    float  rela_y = 0.0f;
    float  rela_z = 0.0f;  // 相对方向
    double lon    = 0.0f;  // 经度
    double lat    = 0.0f;  // 纬度
    double elev   = 0.0f;  // 海拔高度
    //
    std::string to_string()
    {
        std::ostringstream oss;
        oss << "Magnetic Field Fingerprint: "
            << "mag_x: " << mag_x << ", "
            << "mag_y: " << mag_y << ", "
            << "mag_z: " << mag_z << ", "
            << "rela_x: " << rela_x << ", "
            << "rela_y: " << rela_y << ", "
            << "rela_z: " << rela_z << ", "
            << "lon: " << lon << ", "
            << "lat: " << lat << ", "
            << "elev: " << elev;
        return oss.str();
    }
};
// 磁场指纹结构
struct EIGEN_MAG_FIELD_FINGERPRINT
{
    std::string c_zoon;
    std::string t_zoon;
    std::string b_zoon;
    std::string e_zoon;
    std::string s_zoon;
    std::string w_zoon;
    std::string n_zoon;
    //
    std::vector< MAG_FIELD_FINGERPRINT > mag_field_fingerprints;

    //
    void findNearest_esd( EIGEN_SENSOR_DATA& esd, float threshold_distance )
    {
        Eigen::VectorXf query_original( 3 );
        query_original << esd.mag[ 0 ], esd.mag[ 1 ], esd.mag[ 2 ];
        // 使用四元数旋转向量
        // 四元数的构造函数为 w, x, y, z
        Eigen::Quaternionf rotationQuaternion( esd.qua[ 3 ], esd.qua[ 0 ], esd.qua[ 1 ], esd.qua[ 2 ] );
        Eigen::Vector3f    query = rotationQuaternion * query_original;
        //
        if ( query.size() != 3 )
        {
            throw std::invalid_argument( "Query vector must be 3D (mag_x, mag_y, mag_z)" );
        }

        if ( mag_field_fingerprints.empty() )
        {
            throw std::out_of_range( "Fingerprint database is empty" );
        }

        size_t nearest_idx  = 0;
        float  min_distance = std::numeric_limits< float >::max();

        for ( size_t i = 0; i < mag_field_fingerprints.size(); ++i )
        {
            const auto&     fp = mag_field_fingerprints[ i ];
            Eigen::Vector3f mag_vector( fp.mag_x, fp.mag_y, fp.mag_z );
            float           distance = ( query.head< 3 >() - mag_vector ).norm();

            if ( distance < min_distance )
            {
                min_distance = distance;
                nearest_idx  = i;
            }
        }
        //
        // printf( "Nearest idx: %zu, min_distance: %f\n", nearest_idx, min_distance );
        if ( min_distance < threshold_distance )
        {
            esd.pos[ 0 ] = mag_field_fingerprints[ nearest_idx ].rela_x;
            esd.pos[ 1 ] = mag_field_fingerprints[ nearest_idx ].rela_y;
            esd.pos[ 2 ] = mag_field_fingerprints[ nearest_idx ].rela_z;
        }
    }
};