//
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
rapidcsv::Document          csv_doc_;
rapidcsv::Document          read_csv_doc_( "data.csv" );
WebSocketServer             server;
xioTechnologiesCalculation  ahrs_calculation_;
static int                  read_csv_row_index = 0;
bool                        is_read_sensor     = false;
SQLite::Database            db( "db.db3", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE );
EIGEN_MAG_FIELD_FINGERPRINT mag_field_fingerprint_;
std::string                 current_region = "0";
//
EIGEN_SENSOR_DATA sensor_data_;
EIGEN_SENSOR_DATA original_sensor_data_;
int               csv_index = 0;
//
MMC56x3  sensor_mmc_;
ICM42670 sensor_imu_;
// 信号处理函数
static void signalHandler_for_gloab( int signum )
{
    std::cout << "收到 Ctrl+C 信号，程序即将退出..." << std::endl;
    // 关闭 CSV 文件
    close_out_csv( csv_doc_ );
    // 退出程序
    exit( signum );
}
// fifo 数据读取回调函数
static void irq_handler( void )
{
    uint32_t    step_count   = 0;
    float       step_cadence = 0;
    const char* activity     = nullptr;
    //
    if ( sensor_imu_.getPedometer( step_count, step_cadence, activity ) == 0 )
    {
        printf( "Step count:%d\n", step_count );
        printf( "Step cadence:%f(steps/sec)\n", step_cadence );
        printf( "activity:%s\n", activity ? activity : "" );
    }
}
//
int main()
{
    server.setPort( port );
    server.start();
    server.str_fusion_config = ahrs_calculation_.GetConfigString();
    std::this_thread::sleep_for( std::chrono::seconds( 3 ) );
    //
    init_out_csv( csv_doc_ );
    //
    create_magnetometer_table( db );
    select_magnetometer_table( db, current_region, mag_field_fingerprint_ );
    // 注册信号处理函数，处理 SIGINT 信号（Ctrl+C 产生的信号）
    std::signal( SIGINT, signalHandler_for_gloab );

    //
    init_sensor( sensor_mmc_, sensor_imu_ );
    //
    while ( server.running_ )
    {
        if ( startsWith( server.commond_, "Setup" ) )
        {
            ahrs_calculation_.ConfigFusion( server.commond_ );
            server.str_fusion_config = ahrs_calculation_.GetConfigString();
            // 清空数据：销毁并重新创建
            csv_doc_  = rapidcsv::Document();
            csv_index = 0;
            init_out_csv( csv_doc_ );
            //
            sensor_data_.ToZero();
            original_sensor_data_.ToZero();
            ahrs_calculation_.ResetInitFusion();
            ahrs_calculation_.ResetInitial();
            init_sensor( sensor_mmc_, sensor_imu_ );
            server.commond_    = "";
            start_dalta_index  = 0;
            read_csv_row_index = 0;
            //
            read_csv_doc_.Load( "data.csv" );
        }
        else if ( startsWith( server.commond_, "ReadCsv" ) )
        {
            start_dalta_index++;
            ahrs_calculation_.start_time = server.start_time;
            //
            sensor_data_.ToFusionZero();
            auto read_ret = read_csv_by_index( read_csv_row_index, read_csv_doc_, sensor_data_ );
            //
            if ( read_ret )
            {
                if ( start_dalta_index > 10 )
                {
                    start_dalta_index = 11;
                    //
                    auto calcula_ret = ahrs_calculation_.One_SolveAnCalculation( &sensor_data_ );
                    //
                    if ( calcula_ret )
                    {
                        std::string command = "AfterCalculation:";
                        command += sensor_data_.to_string();
                        // printf("%s\n",command.c_str());
                        server.handleSend( command );
                    }
                    // Run @ ODR 100Hz:10
                    std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
                }
            }
            //
        }

        else if ( startsWith( server.commond_, "Start" ) )
        {
            start_dalta_index++;
            ahrs_calculation_.start_time = server.start_time;
            //
            sensor_data_.ToFusionZero();
            original_sensor_data_.ToFusionZero();
            //
            bool read_ret = read_sensor_data( sensor_mmc_, sensor_imu_, sensor_data_, original_sensor_data_ );
            if ( read_ret )
            {
                if ( start_dalta_index > 10 )
                {
                    start_dalta_index = 11;
                    //
                    auto calcula_ret = ahrs_calculation_.Mul_SolveAnCalculation( &sensor_data_, &original_sensor_data_ );
                    //
                    if ( calcula_ret )
                    {
                        //
                        // mag_field_fingerprint_.findNearest_esd( sensor_data_, 10.0f );
                        //
                        std::string command = "AfterCalculation:|";
                        command += sensor_data_.to_json_string();
                        // 
                        EIGEN_SENSOR_DATA xxx;
                        xxx.getValueFromJsonString( command );

                        server.handleSend( command );
                        //
                        command = "BeforCalculation:|";
                        command += original_sensor_data_.to_json_string();
                        server.handleSend( command );
                        //
                        if ( csv_index < 100000 )
                        {
                            update_out_csv( csv_index, csv_doc_, sensor_data_ );
                        }
                    }
                    // Run @ ODR 100Hz:10
                    std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
                }
            }
        }
        else if ( startsWith( server.commond_, "FifoStart" ) )
        {
            start_dalta_index++;
            ahrs_calculation_.start_time = server.start_time;
            //
            sensor_data_.ToZero();
            original_sensor_data_.ToZero();
            // 读取 FIFO 数据
            ahrs_calculation_.ResetInitFusion();
            ahrs_calculation_.ResetInitial();
            //
            init_sensor( sensor_mmc_, sensor_imu_ );
            // Enable interrupt on pin 17, Fifo watermark=10
            sensor_imu_.startPedometer( 17, irq_handler );
            //
            // sensor_imu_.monitor.detach();
            //
            server.commond_ = "";
        }
        else if ( startsWith( server.commond_, "Fingerprint" ) )
        {
            char  delimiter = ',';
            auto  values    = splitString( server.commond_, delimiter );
            float x = 0.0f, y = 0.0f, z = 0.0f;
            //
            if ( values.size() >= 10 )
            {
                // 解析指纹数据
                x = std::stof( values[ 1 ] );
                y = std::stof( values[ 2 ] );
                z = std::stof( values[ 3 ] );
                //
                insert_magnetometer_table( db, sensor_data_, x, y, z, values[ 4 ], values[ 5 ], values[ 6 ], values[ 7 ], values[ 8 ], values[ 9 ], values[ 10 ] );
                //
                select_magnetometer_table( db, current_region, mag_field_fingerprint_ );
            }
            //
            server.commond_ = "Start";
        }
        else if ( startsWith( server.commond_, "Pause" ) )
        {
            close_out_csv( csv_doc_ );
            csv_index          = 0;
            server.commond_    = "";
            start_dalta_index  = 0;
            read_csv_row_index = 0;
        }
        else if ( startsWith( server.commond_, "Reset" ) )
        {
            // 清空数据：销毁并重新创建
            csv_doc_  = rapidcsv::Document();
            csv_index = 0;
            init_out_csv( csv_doc_ );
            //
            sensor_data_.ToZero();
            original_sensor_data_.ToZero();
            ahrs_calculation_.ResetInitial();
            init_sensor( sensor_mmc_, sensor_imu_ );
            start_dalta_index  = 0;
            read_csv_row_index = 0;
            //
            read_csv_doc_.Load( "data.csv" );
        }
        else if ( startsWith( server.commond_, "Clear" ) )
        {
            system( "clear" );
            start_dalta_index = 0;
        }
        else if ( startsWith( server.commond_, "Stop" ) )
        {
            server.commond_   = "";
            start_dalta_index = 0;
            server.stop();
        }
    }
    //
    server.stop();
    close_out_csv( csv_doc_ );
    //
    return 0;
}
