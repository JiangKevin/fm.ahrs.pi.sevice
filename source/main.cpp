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
    server.str_fusion_config = ahrs_calculation_.GetConfigString();
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
    SENSOR_DB original_sensor_data_;
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
            server.commond_   = "";
            start_dalta_index = 0;
        }
        else if ( startsWith( server.commond_, "ReadCsv" ) )
        {
            static int row_index = 0;
            //
            auto ret = read_csv_by_index( row_index, read_csv_doc_, sensor_data_ );
            //
            if ( ret )
            {
                std::string command = "AfterCalculation:";
                command += sensor_data_.to_string();
                // printf("%s\n",command.c_str());
                server.handleSend( command );
            }
            //
        }
        else if ( startsWith( server.commond_, "Start" ) )
        {
            start_dalta_index++;
            ahrs_calculation_.start_time = server.start_time;

            //
            bool read_ret = read_sensor_data( sensor_mmc_, sensor_imu_, sensor_data_, original_sensor_data_ );
            if ( read_ret )
            {
                if ( start_dalta_index > 10 )
                {
                    start_dalta_index = 11;
                    //
                    auto calcula_ret = ahrs_calculation_.SolveAnCalculation( &sensor_data_, &original_sensor_data_ );
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
                        if ( index < 10000 )
                        {
                            update_out_csv( index, csv_doc_, sensor_data_ );
                        }
                    }
                    // Run @ ODR 100Hz:10
                    std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
                }
            }
        }
        else if ( startsWith( server.commond_, "Pause" ) )
        {
            close_out_csv( csv_doc_ );
            index             = 0;
            server.commond_   = "";
            start_dalta_index = 0;
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
            server.commond_   = "Start";
            start_dalta_index = 0;
        }
        else if ( startsWith( server.commond_, "Clear" ) )
        {
            system( "clear" );
            server.commond_   = "Start";
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
