

void AhrsCalculation::filterAccelerationWithMagAndLp( float ax, float ay, float az, float mx, float my, float mz, float& out_ax, float& out_ay, float& out_az, float threshold_x, float threshold_y, float threshold_z )
{
    // 使用 static 保持每个轴低通滤波器的状态（以便多次调用后保持历史平滑效果）
    static LowPassFilter filterX( 0.8f );
    static LowPassFilter filterY( 0.8f );
    static LowPassFilter filterZ( 0.8f );

    // 分别对输入的净加速度执行低通滤波
    float fx = filterX.filter( ax );
    float fy = filterY.filter( ay );
    float fz = filterZ.filter( az );
    // 根据各轴阈值判断：低于阈值视为噪声，输出 0；否则输出滤波（及调整）后的结果
    out_ax = ( std::fabs( ax ) < threshold_x ) ? 0.0f : fx;
    out_ay = ( std::fabs( ay ) < threshold_y ) ? 0.0f : fy;
    out_az = ( std::fabs( az ) < threshold_z ) ? 0.0f : fz;
}