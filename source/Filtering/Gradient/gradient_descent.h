#include "Calculation/sensor_db.h"
#include <cstdint>
#include <iostream>

// 定义历史数据窗口的大小（这决定了平滑的时间范围）
const int WINDOW_SIZE = 20;
// 内部静态的滑动窗口变量，用于保存历史测量
static std::vector< SENSOR_DB > sensorWindow;

///
/// 使用梯度下降对滑动窗口内的 eacc 数据进行平滑；
/// 平滑结果用于更新当前 sensorData 对应的加速度值。
///
static void smoothAcceleration( SENSOR_DB* sensorData, int iterations = 50, float learningRate = 0.01f, float lambda = 0.1f )
{
    if ( sensorData == nullptr )
    {
        return;
    }
    // 将最新数据追加到窗口中
    sensorWindow.push_back( *sensorData );
    if ( sensorWindow.size() > WINDOW_SIZE )
    {
        sensorWindow.erase( sensorWindow.begin() );  // 删除最老数据
    }

    // 如果窗口内数据不足，暂不平滑
    if ( sensorWindow.size() < WINDOW_SIZE )
    {
        return;
    }

    int n = sensorWindow.size();
    // 用于存储平滑后的值
    std::vector< float > s_x( n ), s_y( n ), s_z( n );
    for ( int i = 0; i < n; ++i )
    {
        s_x[ i ] = sensorWindow[ i ].eacc_x;
        s_y[ i ] = sensorWindow[ i ].eacc_y;
        s_z[ i ] = sensorWindow[ i ].eacc_z;
    }

    // 使用梯度下降算法对整个窗口数据进行平滑
    for ( int iter = 0; iter < iterations; ++iter )
    {
        std::vector< float > grad_x( n, 0.0f ), grad_y( n, 0.0f ), grad_z( n, 0.0f );
        for ( int i = 0; i < n; ++i )
        {
            // 数据拟合项：鼓励平滑值接近原始值
            grad_x[ i ] += 2.0f * ( s_x[ i ] - sensorWindow[ i ].eacc_x );
            grad_y[ i ] += 2.0f * ( s_y[ i ] - sensorWindow[ i ].eacc_y );
            grad_z[ i ] += 2.0f * ( s_z[ i ] - sensorWindow[ i ].eacc_z );

            // 平滑约束项：对相邻数据差分施加惩罚
            if ( i > 0 )
            {
                grad_x[ i ] += 2.0f * lambda * ( s_x[ i ] - s_x[ i - 1 ] );
                grad_y[ i ] += 2.0f * lambda * ( s_y[ i ] - s_y[ i - 1 ] );
                grad_z[ i ] += 2.0f * lambda * ( s_z[ i ] - s_z[ i - 1 ] );
            }
            if ( i < n - 1 )
            {
                grad_x[ i ] += -2.0f * lambda * ( s_x[ i + 1 ] - s_x[ i ] );
                grad_y[ i ] += -2.0f * lambda * ( s_y[ i + 1 ] - s_y[ i ] );
                grad_z[ i ] += -2.0f * lambda * ( s_z[ i + 1 ] - s_z[ i ] );
            }
        }

        // 更新平滑后的值
        for ( int i = 0; i < n; ++i )
        {
            s_x[ i ] -= learningRate * grad_x[ i ];
            s_y[ i ] -= learningRate * grad_y[ i ];
            s_z[ i ] -= learningRate * grad_z[ i ];
        }
    }

    // 将平滑结果应用到当前测量上（可选择用整个窗口内最后一个点的平滑值）
    sensorData->eacc_x = s_x[ n - 1 ];
    sensorData->eacc_y = s_y[ n - 1 ];
    sensorData->eacc_z = s_z[ n - 1 ];
}