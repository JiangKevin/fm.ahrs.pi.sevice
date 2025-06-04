#pragma once
//
#include "Calculation/FromXioTechnologies/xioTechnologiesCalculation.h"
#include "Calculation/comput.h"
#include "Calculation/sensor_db.h"
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
#include "concurrentqueue/concurrentqueue.h"
#include "websocket/websocket_server.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <SQLiteCpp/SQLiteCpp.h>
#include <csignal>
#include <iostream>
#include <rapidcsv.h>
#include <string>
#include <string_view>
#include <unistd.h>
//
using namespace GeographicLib;
//
const std::string i2cDevice         = "/dev/i2c-1";
uint8_t           deviceAddress_mmc = 0x30;
uint8_t           deviceAddress_imu = 0x69;
std::string       host              = "127.0.0.1";
std::string       port              = "18080";
//
static void init_out_csv( rapidcsv::Document& csv_doc )
{
    //
    csv_doc.SetColumnName( 0, "Time (s)" );
    //
    csv_doc.SetColumnName( 1, "Gyroscope X (deg/s)" );
    csv_doc.SetColumnName( 2, "Gyroscope Y (deg/s)" );
    csv_doc.SetColumnName( 3, "Gyroscope Z (deg/s)" );
    //
    csv_doc.SetColumnName( 4, "Accelerometer X (g)" );
    csv_doc.SetColumnName( 5, "Accelerometer Y (g)" );
    csv_doc.SetColumnName( 6, "Accelerometer Z (g)" );
    //
    csv_doc.SetColumnName( 7, "Magnetometer X (uT)" );
    csv_doc.SetColumnName( 8, "Magnetometer Y (uT)" );
    csv_doc.SetColumnName( 9, "Magnetometer Z (uT)" );
    //
    csv_doc.SetColumnName( 10, "quate X (d)" );
    csv_doc.SetColumnName( 11, "quate Y (d)" );
    csv_doc.SetColumnName( 12, "quate Z (d)" );
    csv_doc.SetColumnName( 13, "quate W (d)" );

    csv_doc.SetColumnName( 14, "Roll (d)" );
    csv_doc.SetColumnName( 15, "Pitch (d)" );
    csv_doc.SetColumnName( 16, "Yaw (d)" );
    //
    csv_doc.SetColumnName( 17, "Estimated Accelerometer X (g)" );
    csv_doc.SetColumnName( 18, "Estimated Accelerometer Y (g)" );
    csv_doc.SetColumnName( 19, "Estimated Accelerometer Z (g)" );
    //
    csv_doc.SetColumnName( 20, "Estimated Velocity X (m/s)" );
    csv_doc.SetColumnName( 21, "Estimated Velocity Y (m/s)" );
    csv_doc.SetColumnName( 22, "Estimated Velocity Z (m/s)" );
    //
    csv_doc.SetColumnName( 23, "Estimated Position X (m)" );
    csv_doc.SetColumnName( 24, "Estimated Position Y (m)" );
    csv_doc.SetColumnName( 25, "Estimated Position Z (m)" );
    csv_doc.SetColumnName( 26, "Delta Time (s)" );
    csv_doc.SetColumnName( 27, "Total Acc (g)" );
}
//
static void update_out_csv( int& index, rapidcsv::Document& csv_doc, const EIGEN_SENSOR_DATA& sensor_data )
{
    // csv_doc.
    // 添加数据行
    csv_doc.SetCell< int64_t >( 0, index, sensor_data.time );
    csv_doc.SetCell< float >( 1, index, sensor_data.gyr[ 0 ] );
    csv_doc.SetCell< float >( 2, index, sensor_data.gyr[ 1 ] );
    csv_doc.SetCell< float >( 3, index, sensor_data.gyr[ 2 ] );
    csv_doc.SetCell< float >( 4, index, sensor_data.acc[ 0 ] );
    csv_doc.SetCell< float >( 5, index, sensor_data.acc[ 1 ] );
    csv_doc.SetCell< float >( 6, index, sensor_data.acc[ 2 ] );
    csv_doc.SetCell< float >( 7, index, sensor_data.mag[ 0 ] );
    csv_doc.SetCell< float >( 8, index, sensor_data.mag[ 1 ] );
    csv_doc.SetCell< float >( 9, index, sensor_data.mag[ 2 ] );
    csv_doc.SetCell< float >( 10, index, sensor_data.qua[ 0 ] );
    csv_doc.SetCell< float >( 11, index, sensor_data.qua[ 1 ] );
    csv_doc.SetCell< float >( 12, index, sensor_data.qua[ 2 ] );
    csv_doc.SetCell< float >( 13, index, sensor_data.qua[ 3 ] );
    csv_doc.SetCell< float >( 14, index, sensor_data.eul[ 0 ] );
    csv_doc.SetCell< float >( 15, index, sensor_data.eul[ 1 ] );
    csv_doc.SetCell< float >( 16, index, sensor_data.eul[ 2 ] );
    csv_doc.SetCell< float >( 17, index, sensor_data.eacc[ 0 ] );
    csv_doc.SetCell< float >( 18, index, sensor_data.eacc[ 1 ] );
    csv_doc.SetCell< float >( 19, index, sensor_data.eacc[ 2 ] );
    csv_doc.SetCell< float >( 20, index, sensor_data.vel[ 0 ] );
    csv_doc.SetCell< float >( 21, index, sensor_data.vel[ 1 ] );
    csv_doc.SetCell< float >( 22, index, sensor_data.vel[ 2 ] );
    csv_doc.SetCell< float >( 23, index, sensor_data.pos[ 0 ] );
    csv_doc.SetCell< float >( 24, index, sensor_data.pos[ 1 ] );
    csv_doc.SetCell< float >( 25, index, sensor_data.pos[ 2 ] );
    csv_doc.SetCell< float >( 26, index, sensor_data.deltaTime );
    csv_doc.SetCell< float >( 27, index, sensor_data.totalAcc );
    //
    index++;
}
//
static bool read_csv_by_index( int& index, rapidcsv::Document& csv_doc, EIGEN_SENSOR_DATA& sensor_data )
{
    int count = csv_doc.GetColumn< int64_t >( "Time (s)" ).size();
    //
    if ( index < count )
    {
        sensor_data.time      = csv_doc.GetCell< int64_t >( 0, index );
        sensor_data.gyr[ 0 ]  = csv_doc.GetCell< float >( 1, index );
        sensor_data.gyr[ 1 ]  = csv_doc.GetCell< float >( 2, index );
        sensor_data.gyr[ 2 ]  = csv_doc.GetCell< float >( 3, index );
        sensor_data.acc[ 0 ]  = csv_doc.GetCell< float >( 4, index );
        sensor_data.acc[ 1 ]  = csv_doc.GetCell< float >( 5, index );
        sensor_data.acc[ 2 ]  = csv_doc.GetCell< float >( 6, index );
        sensor_data.mag[ 0 ]  = csv_doc.GetCell< float >( 7, index );
        sensor_data.mag[ 1 ]  = csv_doc.GetCell< float >( 8, index );
        sensor_data.mag[ 2 ]  = csv_doc.GetCell< float >( 9, index );
        sensor_data.qua[ 0 ]  = csv_doc.GetCell< float >( 10, index );
        sensor_data.qua[ 1 ]  = csv_doc.GetCell< float >( 11, index );
        sensor_data.qua[ 2 ]  = csv_doc.GetCell< float >( 12, index );
        sensor_data.qua[ 3 ]  = csv_doc.GetCell< float >( 13, index );
        sensor_data.eul[ 0 ]  = csv_doc.GetCell< float >( 14, index );
        sensor_data.eul[ 1 ]  = csv_doc.GetCell< float >( 15, index );
        sensor_data.eul[ 2 ]  = csv_doc.GetCell< float >( 16, index );
        sensor_data.eacc[ 0 ] = csv_doc.GetCell< float >( 17, index );
        sensor_data.eacc[ 1 ] = csv_doc.GetCell< float >( 18, index );
        sensor_data.eacc[ 2 ] = csv_doc.GetCell< float >( 19, index );
        sensor_data.vel[ 0 ]  = csv_doc.GetCell< float >( 20, index );
        sensor_data.vel[ 1 ]  = csv_doc.GetCell< float >( 21, index );
        sensor_data.vel[ 2 ]  = csv_doc.GetCell< float >( 22, index );
        sensor_data.pos[ 0 ]  = csv_doc.GetCell< float >( 23, index );
        sensor_data.pos[ 1 ]  = csv_doc.GetCell< float >( 24, index );
        sensor_data.pos[ 2 ]  = csv_doc.GetCell< float >( 25, index );
        sensor_data.deltaTime = csv_doc.GetCell< float >( 26, index );
        sensor_data.totalAcc  = csv_doc.GetCell< float >( 27, index );
        //
        index++;
        //
        return true;
    }
    //
    return false;
}
//
static void close_out_csv( rapidcsv::Document& csv_doc )
{
    try
    {
        csv_doc.Save( "out.csv" );
        std::cout << "数据已成功写入 output.csv 文件。" << std::endl;
    }
    catch ( const std::exception& e )
    {
        std::cerr << "写入文件时出错: " << e.what() << std::endl;
    }
}
//
static void init_sensor( MMC56x3& sensor_mmc, ICM42670& sensor_imu )
{
    // Initializing the MMC56x3
    if ( ! sensor_mmc.begin( deviceAddress_mmc, i2cDevice.c_str() ) )
    {
        printf( "Failed to initialize MMC56x3 sensor\n" );
    }
    // Initializing the ICM42670
    int ret;
    ret = sensor_imu.begin( false, deviceAddress_imu, i2cDevice.c_str() );
    if ( ret != 0 )
    {
        printf( "Failed to initialize ICM42670 sensor\n" );
    }
    // Accel ODR = 100 Hz and Full Scale Range = 16G
    sensor_imu.startAccel( 100, 16 );
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    sensor_imu.startGyro( 100, 2000 );
    // Wait IMU to start
    std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
}
//
static bool read_sensor_data( MMC56x3& sensor_mmc, ICM42670& sensor_imu, EIGEN_SENSOR_DATA& sensor_data, EIGEN_SENSOR_DATA& original_sensor_data )
{
    //
    sensor_data.time = getMicrosecondTimestamp();
    // printf( "Time: %ld\n", sensor_data.time );
    // MMC56x3
    float x, y, z;
    if ( sensor_mmc.getEvent( x, y, z ) )
    {
        sensor_data.mag[ 0 ] = x;
        sensor_data.mag[ 1 ] = y;
        sensor_data.mag[ 2 ] = z;
        //
        original_sensor_data.mag[ 0 ] = x;
        original_sensor_data.mag[ 1 ] = y;
        original_sensor_data.mag[ 2 ] = z;
    }
    else
    {
        return false;
    }
    float temp = sensor_mmc.readTemperature();
    if ( std::isnan( temp ) )
    {
        return false;
    }
    // TDK42607
    inv_imu_sensor_event_t imu_event;
    // Get last event
    sensor_imu.getDataFromRegisters( imu_event );
    //
    sensor_data.acc[ 0 ] = imu_event.accel[ 0 ] / 2048.0;
    sensor_data.acc[ 1 ] = imu_event.accel[ 1 ] / 2048.0;
    sensor_data.acc[ 2 ] = imu_event.accel[ 2 ] / 2048.0;
    sensor_data.gyr[ 0 ] = imu_event.gyro[ 0 ] / 16.4;
    sensor_data.gyr[ 1 ] = imu_event.gyro[ 1 ] / 16.4;
    sensor_data.gyr[ 2 ] = imu_event.gyro[ 2 ] / 16.4;
    //
    original_sensor_data.acc[ 0 ] = imu_event.accel[ 0 ] / 2048.0;
    original_sensor_data.acc[ 1 ] = imu_event.accel[ 1 ] / 2048.0;
    original_sensor_data.acc[ 2 ] = imu_event.accel[ 2 ] / 2048.0;
    original_sensor_data.gyr[ 0 ] = imu_event.gyro[ 0 ] / 16.4;
    original_sensor_data.gyr[ 1 ] = imu_event.gyro[ 1 ] / 16.4;
    original_sensor_data.gyr[ 2 ] = imu_event.gyro[ 2 ] / 16.4;

    //
    return true;
}
//
static void create_magnetometer_table( SQLite::Database& db )
{
    try
    {
        SQLite::Statement query( db, "CREATE TABLE IF NOT EXISTS magnetometer ("
                                     "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                                     "time TEXT, "
                                     "mag_x TEXT, "
                                     "mag_y TEXT, "
                                     "mag_z TEXT, "
                                     "rela_x TEXT, "
                                     "rela_y TEXT, "
                                     "rela_z TEXT, "
                                     "lon TEXT, "
                                     "lat TEXT, "
                                     "elev TEXT "
                                     ");" );
        query.exec();
    }
    catch ( const SQLite::Exception& e )
    {
        std::cerr << "SQLite error: " << e.what() << std::endl;
    }
}
//
static void insert_magnetometer_table( SQLite::Database& db, EIGEN_SENSOR_DATA& sensor_data, float rela_x, float rela_y, float rela_z )
{
    // 当前点的经纬度和高度，作为局部坐标系的原点
    double            origin_latitude  = 29.116543;   // 纬度
    double            origin_longitude = 111.506270;  // 经度
    double            origin_height    = 0.0;         // 高度
    const Geocentric& earth            = Geocentric::WGS84();
    LocalCartesian    proj( origin_latitude, origin_longitude, origin_height, earth );
    //
    double lon, lat, elev;
    // 转换为局部直角坐标系
    proj.Forward( rela_y, rela_x, rela_z, lon, lat, elev );
    //
    try
    {
        SQLite::Statement query( db, "INSERT INTO magnetometer (time, mag_x, mag_y, mag_z, rela_x, rela_y, rela_z, lon, lat, elev) "
                                     "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?);" );
        // 这里需要填入实际的值
        query.bind( 1, std::to_string( sensor_data.time ) );      // 示例时间
        query.bind( 2, std::to_string( sensor_data.mag[ 0 ] ) );  // 示例磁力计数据
        query.bind( 3, std::to_string( sensor_data.mag[ 1 ] ) );
        query.bind( 4, std::to_string( sensor_data.mag[ 2 ] ) );
        query.bind( 5, std::to_string( rela_x ) );  // 示例相对方向数据
        query.bind( 6, std::to_string( rela_y ) );
        query.bind( 7, std::to_string( rela_z ) );
        query.bind( 8, std::to_string( lon ) );    // 示例经度
        query.bind( 9, std::to_string( lat ) );    // 示例纬度
        query.bind( 10, std::to_string( elev ) );  // 示例海拔高度
        query.exec();
    }
    catch ( const SQLite::Exception& e )
    {
        std::cerr << "SQLite error: " << e.what() << std::endl;
    }
}