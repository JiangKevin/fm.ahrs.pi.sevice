//
#include "Calculation/AhrsCalculation.h"
#include "MMC56x3/MMC56x3.h"
#include "TDK40607P/ICM42670P.h"
#include "concurrentqueue/concurrentqueue.h"
#include "g_fun.h"
#include "websocket/websocket_server.hpp"
#include <csignal>
#include <iostream>
#include <rapidcsv.h>
#include <string>
#include <string_view>
#include <unistd.h>

//
rapidcsv::Document csv_doc_;
WebSocketServer    server;
AhrsCalculation    ahrs_calculation_;

// 信号处理函数
static void signalHandler_for_gloab( int signum )
{
    std::cout << "收到 Ctrl+C 信号，程序即将退出..." << std::endl;
    // 关闭 CSV 文件
    close_out_csv( csv_doc_ );
    // 退出程序
    exit( signum );
}
//
int main()
{
    server.setPort( port );
    server.start();
    std::this_thread::sleep_for( std::chrono::seconds( 3 ) );
    //
    init_out_csv( csv_doc_ );
    int index = 0;
    // 注册信号处理函数，处理 SIGINT 信号（Ctrl+C 产生的信号）
    std::signal( SIGINT, signalHandler_for_gloab );
    //
    MMC56x3  sensor_mmc_;
    ICM42670 sensor_imu_;
    //
    SENSOR_DB sensor_data_;
    //
    init_sensor( sensor_mmc_, sensor_imu_ );
    //
    // server.handleSend
    //
    while ( server.running_ )
    {
        if ( startsWith( server.commond_, "Setup" ) )
        {
            char delimiter = ',';
            auto values    = splitString( server.commond_, delimiter );

            //
            if ( values.size() == 49 )
            {
                // printf( "%s\n", server.commond_.c_str() );
                //
                ahrs_calculation_.gyroscopeMisalignment = { std::stof( values[ 1 ] ), std::stof( values[ 2 ] ), std::stof( values[ 3 ] ), std::stof( values[ 4 ] ), std::stof( values[ 5 ] ), std::stof( values[ 6 ] ), std::stof( values[ 7 ] ), std::stof( values[ 8 ] ), std::stof( values[ 9 ] ) };
                //
                ahrs_calculation_.gyroscopeSensitivity = { std::stof( values[ 10 ] ), std::stof( values[ 11 ] ), std::stof( values[ 12 ] ) };
                //
                ahrs_calculation_.gyroscopeOffset = { std::stof( values[ 13 ] ), std::stof( values[ 14 ] ), std::stof( values[ 15 ] ) };
                //
                ahrs_calculation_.accelerometerMisalignment = { std::stof( values[ 16 ] ), std::stof( values[ 17 ] ), std::stof( values[ 18 ] ), std::stof( values[ 19 ] ), std::stof( values[ 20 ] ),
                                                                std::stof( values[ 21 ] ), std::stof( values[ 22 ] ), std::stof( values[ 23 ] ), std::stof( values[ 24 ] ) };
                //
                ahrs_calculation_.accelerometerSensitivity = { std::stof( values[ 25 ] ), std::stof( values[ 26 ] ), std::stof( values[ 27 ] ) };
                ahrs_calculation_.accelerometerOffset      = { std::stof( values[ 28 ] ), std::stof( values[ 29 ] ), std::stof( values[ 30 ] ) };
                //
                ahrs_calculation_.softIronMatrix = { std::stof( values[ 31 ] ), std::stof( values[ 32 ] ), std::stof( values[ 33 ] ), std::stof( values[ 34 ] ), std::stof( values[ 35 ] ), std::stof( values[ 36 ] ), std::stof( values[ 37 ] ), std::stof( values[ 38 ] ), std::stof( values[ 39 ] ) };
                //
                ahrs_calculation_.hardIronOffset = { std::stof( values[ 40 ] ), std::stof( values[ 41 ] ), std::stof( values[ 42 ] ) };
                //

                //
                if ( std::stoi( values[ 43 ] ) == 0 )
                {
                    ahrs_calculation_.settings.convention = FusionConventionNwu;
                }
                else if ( std::stoi( values[ 43 ] ) == 1 )
                {
                    ahrs_calculation_.settings.convention = FusionConventionEnu;
                }
                else if ( std::stoi( values[ 43 ] ) == 2 )
                {
                    ahrs_calculation_.settings.convention = FusionConventionNed;
                }
                //
                ahrs_calculation_.settings.gain                  = std::stof( values[ 44 ] );
                ahrs_calculation_.settings.gyroscopeRange        = std::stof( values[ 45 ] );
                ahrs_calculation_.settings.accelerationRejection = std::stof( values[ 46 ] );
                ahrs_calculation_.settings.magneticRejection     = std::stof( values[ 47 ] );
                ahrs_calculation_.settings.recoveryTriggerPeriod = std::stoul( values[ 48 ] ) * SAMPLE_RATE;
            }
            //
            // 清空数据：销毁并重新创建
            csv_doc_ = rapidcsv::Document();
            index    = 0;
            init_out_csv( csv_doc_ );
            //
            ahrs_calculation_.ResetInitFusion();
            ahrs_calculation_.ResetInitial();
            init_sensor( sensor_mmc_, sensor_imu_ );
            server.commond_ = "";
        }
        //
        if ( server.commond_ == "Start" )
        {
            bool ret = read_sensor_data( sensor_mmc_, sensor_imu_, ahrs_calculation_, sensor_data_ );
            if ( ret )
            {
                server.handleSend( sensor_data_.to_string() );
                //
                update_out_csv( index, csv_doc_, sensor_data_ );
            }
        }
        else if ( server.commond_ == "Pause" )
        {
            close_out_csv( csv_doc_ );
            index           = 0;
            server.commond_ = "";
        }
        else if ( server.commond_ == "Reset" )
        {
            // 清空数据：销毁并重新创建
            csv_doc_ = rapidcsv::Document();
            index    = 0;
            init_out_csv( csv_doc_ );
            //
            ahrs_calculation_.ResetInitial();
            init_sensor( sensor_mmc_, sensor_imu_ );
            server.commond_ = "Start";
        }
        else if ( server.commond_ == "Clear" )
        {
            system( "clear" );
            server.commond_ = "Start";
        }
        else if ( server.commond_ == "Stop" )
        {
            server.commond_ = "";
            server.stop();
        }
    }
    //
    server.stop();
    close_out_csv( csv_doc_ );
    //
    return 0;
}
