#include <cmath>
#include <cstdlib>
#include <iostream>

// 单轴低通滤波器类
class LowPassFilter
{
public:
    // 构造函数，alpha 为平滑因子（0~1），alpha 越小滤波越平滑、响应越慢
    LowPassFilter( float alpha = 0.1f ) : alpha_( alpha ), inited_( false ), filtered_value_( 0.0f ) {}

    float filter( float input )
    {
        if ( ! inited_ )
        {
            filtered_value_ = input;
            inited_         = true;
        }
        else
        {
            filtered_value_ = alpha_ * input + ( 1 - alpha_ ) * filtered_value_;
        }
        return filtered_value_;
    }
private:
    float alpha_;
    bool  inited_;
    float filtered_value_;
};