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
rapidcsv::Document         csv_doc_;
rapidcsv::Document         read_csv_doc_( "data.csv" );
WebSocketServer            server;
xioTechnologiesCalculation ahrs_calculation_;
static int                 read_csv_row_index = 0;
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
//
static void event_cb( inv_imu_sensor_event_t* evt )
{
    // Format data for Serial Plotter
    if ( sensor_imu_.isAccelDataValid( evt ) && sensor_imu_.isGyroDataValid( evt ) )
    {
        // Format data for Serial Plotter
        printf( "Time:%u,", evt->timestamp_fsync );
        printf( "AccelX:%f,", evt->accel[ 0 ] / 2048.0 );
        printf( "AccelY:%f,", evt->accel[ 1 ] / 2048.0 );
        printf( "AccelZ:%f,", evt->accel[ 2 ] / 2048.0 );
        printf( "GyroX:%f,", evt->gyro[ 0 ] / 16.4 );
        printf( "GyroY:%f,", evt->gyro[ 1 ] / 16.4 );
        printf( "GyroZ:%f,", evt->gyro[ 2 ] / 16.4 );
        printf( "Temperature:%f", ( evt->temperature / 128.0 ) + 25.0 );
        printf( "\n" );
    }
}
// fifo 数据读取回调函数
static void irq_handler( void )
{
    sensor_imu_.getDataFromFifo( event_cb );
    //
    uint32_t    step_count   = 0;
    float       step_cadence = 0;
    const char* activity     = nullptr;

    if ( sensor_imu_.getPedometer( step_count, step_cadence, activity ) == 0 )
    {
        printf( "Step count:%d\n", step_count );
        printf( "Step cadence:%f(steps/sec)\n", step_cadence );
        printf( "activity:%s\n", activity ? activity : "" );
    }
    if ( sensor_imu_.getTilt() )
    {
        printf( "TILT:%s\n", "true" );
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
    int index = 0;
    // 注册信号处理函数，处理 SIGINT 信号（Ctrl+C 产生的信号）
    std::signal( SIGINT, signalHandler_for_gloab );

    //
    EIGEN_SENSOR_DATA sensor_data_;
    EIGEN_SENSOR_DATA original_sensor_data_;
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
            csv_doc_ = rapidcsv::Document();
            index    = 0;
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
                        std::string command = "AfterCalculation:";
                        command += sensor_data_.to_string();
                        // printf("%s\n",command.c_str());
                        server.handleSend( command );
                        //
                        command = "BeforCalculation:";
                        command += original_sensor_data_.to_string();
                        server.handleSend( command );
                        //
                        if ( index < 100000 )
                        {
                            update_out_csv( index, csv_doc_, sensor_data_ );
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
            sensor_data_.ToFusionZero();
            original_sensor_data_.ToFusionZero();
            // 读取 FIFO 数据
            ahrs_calculation_.ResetInitFusion();
            ahrs_calculation_.ResetInitial();
            //
            int ret = sensor_imu_.begin( false, ICM42670_I2C_ADDRESS, "/dev/i2c-1" );
            if ( ret != 0 )
            {
                perror( "ICM42670 initialization failed: " );
                return -1;
            }
            // Enable interrupt on pin 17, Fifo watermark=10
            sensor_imu_.enableFifoInterrupt( 17, irq_handler, 10 );
            // Accel ODR = 100 Hz and Full Scale Range = 16G
            sensor_imu_.startAccel( 100, 16 );
            // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
            sensor_imu_.startGyro( 100, 2000 );
            // Pedometer enabled
            sensor_imu_.startPedometer();
            // Tilt enabled
            sensor_imu_.startTiltDetection();
            //
            sensor_imu_.monitor.detach();
            //
            server.commond_ = "";
        }
        else if ( startsWith( server.commond_, "Pause" ) )
        {
            close_out_csv( csv_doc_ );
            index              = 0;
            server.commond_    = "";
            start_dalta_index  = 0;
            read_csv_row_index = 0;
        }
        else if ( startsWith( server.commond_, "Reset" ) )
        {
            // 清空数据：销毁并重新创建
            csv_doc_ = rapidcsv::Document();
            index    = 0;
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
