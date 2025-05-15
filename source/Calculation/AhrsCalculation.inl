//
void AhrsCalculation::UseKF( SENSOR_DB* sensor_data, float dt )
{
    // 模拟真实加速度（理想数据，无噪声），例如恒定加速度 [1.5, -2.0, 0.5] m/s²
    Eigen::VectorXf true_acc( 3 );
    true_acc << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
    // 1. 预测步骤
    akf.predict();
    // Eigen::VectorXf predicted = akf.getState();
    // std::cout << "predicted:\t" << predicted.transpose() << "\n";

    // 2. 测量步骤：这里直接用理想真实加速度
    // std::cout << "true_acc:\t" << true_acc.transpose() << "\n";

    // 3. 更新步骤
    akf.update( true_acc );
    // Eigen::VectorXf updated = akf.getState();

    // 输出预测、测量和更新后的加速度（单位：m/s²）
    // std::cout << "updated:\t" << updated.transpose() << "\n";
    //
    Eigen::VectorXf result = akf.getState();

    // 分离出加速度与偏置估计
    Eigen::VectorXf est_acc  = result.head( 3 );
    Eigen::VectorXf est_bias = result.tail( 3 );
    //
    sensor_data->eacc_x = est_acc[ 0 ];
    sensor_data->eacc_y = est_acc[ 1 ];
    sensor_data->eacc_z = est_acc[ 2 ];
}
//
void AhrsCalculation ::initKF()
{
    // 初始化滤波器：
    // 时间步长 dt = 0.065 s；
    // 初始加速度：假设初始为 [0, 0, 0] m/s²；
    // 过程噪声设为 0.001，观测噪声设为 1e-6（理想数据，噪声极小）
    // 模拟时间步长（秒）
    float dt = 0.065;
    //
    Eigen::VectorXf initial_acc( 3 );
    initial_acc << 0.0f, 0.0f, 0.0f;

    Eigen::VectorXf initial_bias( 3 );
    // 例如，通过校准获得：x 轴偏置 0.02, y 轴 -0.01, z 轴 0.03
    // initial_bias << -0.01f, -0.01f, 0.025f;
    initial_bias << 0.0f, 0.0f, -0.002f;

    // 设定过程噪声（加速度部分较大，偏置部分较小）与观测噪声
    float process_noise_acc  = 0.1f;
    float process_noise_bias = 0.001f;
    float measurement_noise  = 0.001f;
    //
    akf.init( dt, initial_acc, initial_bias, process_noise_acc, process_noise_bias, measurement_noise );
    //
    // akf.init( 0.065f, initial_acc, 0.1, 0.001 );
}
//
// 修改后的函数：采用低通滤波器对三个轴的加速度进行滤波，
// 当经过滤波后的每个轴如果低于各自的阈值时，认为该轴输出 0；
// 函数接口增加三个阈值参数：threshold_x, threshold_y, threshold_z
void AhrsCalculation::filterAcceleration( float ax, float ay, float az, float& out_ax, float& out_ay, float& out_az, float threshold_x, float threshold_y, float threshold_z )
{
    // 使用 static 保证低通滤波器状态在多次调用间保持
    static LowPassFilter filterX( 1.0f );
    static LowPassFilter filterY( 1.0f );
    static LowPassFilter filterZ( 1.0f );

    // 对各个轴进行低通滤波
    float fx = filterX.filter( ax );
    float fy = filterY.filter( ay );
    float fz = filterZ.filter( az );

    // 对每个分量分别判断是否低于设定的阈值，
    // 如果低于则认为该轴没有有效运动信号，输出 0；否则输出滤波后的结果
    out_ax = ( std::fabs( fx ) < threshold_x ) ? 0.0f : fx;
    out_ay = ( std::fabs( fy ) < threshold_y ) ? 0.0f : fy;
    out_az = ( std::fabs( fz ) < threshold_z ) ? 0.0f : fz;
}
//
// 扩展后的函数：除了对加速度xyz进行低通滤波外，加入磁力计数据(mx,my,mz)计算设备姿态并剔除重力分量
// 参数threshold_x, threshold_y, threshold_z为各轴阈值，低于阈值时置为0
void AhrsCalculation::filterAccelerationWithMagnetometer( float ax, float ay, float az, float mx, float my, float mz, float& out_ax, float& out_ay, float& out_az, float threshold_x, float threshold_y, float threshold_z )
{
    // 低通滤波器（采用 static 保持状态）
    static LowPassFilter filterX( 0.1f );
    static LowPassFilter filterY( 0.1f );
    static LowPassFilter filterZ( 0.1f );

    float fx = filterX.filter( ax );
    float fy = filterY.filter( ay );
    float fz = filterZ.filter( az );

    // 计算 roll 与 pitch（单位：弧度）
    float roll  = std::atan2( fy, fz );
    float pitch = std::atan2( -fx, std::sqrt( fy * fy + fz * fz ) );

    // 利用磁力计计算 yaw（倾斜补偿后的磁力计数据）
    float mx_comp = mx * std::cos( pitch ) + mz * std::sin( pitch );
    float my_comp = mx * std::sin( roll ) * std::sin( pitch ) + my * std::cos( roll ) - mz * std::sin( roll ) * std::cos( pitch );
    float yaw     = std::atan2( -my_comp, mx_comp );
    // yaw 可用于进一步的姿态修正，但本示例主要利用 roll 和 pitch 来计算重力分量

    // 假设重力加速度（单位：m/s^2）
    const float g = 9.81f;
    // 计算重力在设备坐标系下的分量（公式根据具体坐标系可能需要调整）
    float g_x = -g * std::sin( pitch );
    float g_y = g * std::sin( roll ) * std::cos( pitch );
    float g_z = g * std::cos( roll ) * std::cos( pitch );

    // 剔除重力分量，得到净加速度
    float net_ax = fx - g_x;
    float net_ay = fy - g_y;
    float net_az = fz - g_z;

    // 根据各轴阈值判断，如果净加速度低于阈值，则认为该轴没有实质性运动，输出 0
    out_ax = ( std::fabs( net_ax ) < threshold_x ) ? 0.0f : net_ax;
    out_ay = ( std::fabs( net_ay ) < threshold_y ) ? 0.0f : net_ay;
    out_az = ( std::fabs( net_az ) < threshold_z ) ? 0.0f : net_az;
}
//
// 修改后的函数：
// 已知 roll, pitch, yaw 后，直接利用 roll 与 pitch 计算重力在各轴分量，
// 再从低通滤波后的加速度中去除重力，获得净加速度。
// 参数 threshold_x, threshold_y, threshold_z 分别为各轴的阈值，当净加速度低于阈值时视为静止，即输出 0
void AhrsCalculation::filterAccelerationWithAngles( float ax, float ay, float az, float roll, float pitch, float yaw, float& out_ax, float& out_ay, float& out_az, float threshold_x, float threshold_y, float threshold_z )
{
    // 利用 static 变量保持每个轴低通滤波器的状态，保证数据平滑
    static LowPassFilter filterX( 0.1f );
    static LowPassFilter filterY( 0.1f );
    static LowPassFilter filterZ( 0.1f );

    // 对原始加速度进行低通滤波
    float fx = filterX.filter( ax );
    float fy = filterY.filter( ay );
    float fz = filterZ.filter( az );

    // 使用已知的 roll 和 pitch 计算重力在传感器坐标系中的分量
    // 注意：这里的公式假设重力在全局坐标系为 (0, 0, g) ,
    // 且传感器的坐标系与惯性坐标系之间的旋转由 (roll, pitch, yaw) 给出。
    // 由于重力与 yaw 无关，因此只用 roll 和 pitch。
    const float g   = 9.81f;
    float       g_x = -g * std::sin( pitch );
    float       g_y = g * std::sin( roll ) * std::cos( pitch );
    float       g_z = g * std::cos( roll ) * std::cos( pitch );

    // 剔除重力成分，得到净加速度
    float net_ax = fx - g_x;
    float net_ay = fy - g_y;
    float net_az = fz - g_z;

    // 根据各轴阈值判断，若低于阈值则输出 0，否则输出净加速度
    out_ax = ( std::fabs( net_ax ) < threshold_x ) ? 0.0f : net_ax;
    out_ay = ( std::fabs( net_ay ) < threshold_y ) ? 0.0f : net_ay;
    out_az = ( std::fabs( net_az ) < threshold_z ) ? 0.0f : net_az;
}
//
// 修改后的函数：
// 当加速度数据已经剔除重力影响，同时已知 yaw 时，采用低通滤波对净加速度进行平滑；
// 然后利用 yaw 计算前向单元向量，将水平加速度分解为前向分量和横向分量，
// 如果前向分量为负则置零（即消除反向加速度）。最后，根据各轴阈值过滤小幅噪声。
void AhrsCalculation::filterAccelerationWithYawNoGravity( float ax, float ay, float az, float yaw, float& out_ax, float& out_ay, float& out_az, float threshold_x, float threshold_y, float threshold_z )
{
    // 使用 static 保持每个轴低通滤波器的状态（以便多次调用后保持历史平滑效果）
    static LowPassFilter filterX( 1.0f );
    static LowPassFilter filterY( 1.0f );
    static LowPassFilter filterZ( 1.0f );

    // 分别对输入的净加速度执行低通滤波
    float fx = filterX.filter( ax );
    float fy = filterY.filter( ay );
    float fz = filterZ.filter( az );

    // 利用已知的 yaw（单位：弧度）计算前向单位向量 (cos(yaw), sin(yaw))
    float cos_yaw = std::cos( yaw );
    float sin_yaw = std::sin( yaw );

    // 计算水平加速度在前向（设备朝向）上的分量
    float forward_component = fx * cos_yaw + fy * sin_yaw;

    // 分解出水平加速度的横向分量（垂直于前向方向）
    float lateral_x = fx - forward_component * cos_yaw;
    float lateral_y = fy - forward_component * sin_yaw;

    // 如果前向分量为负，则说明出现反向加速度，将其置为 0
    if ( forward_component < 0 )
        forward_component = 0.0f;

    // 重构调整后的水平加速度
    float adjusted_fx = lateral_x + forward_component * cos_yaw;
    float adjusted_fy = lateral_y + forward_component * sin_yaw;

    // 根据各轴阈值判断：低于阈值视为噪声，输出 0；否则输出滤波（及调整）后的结果
    out_ax = ( std::fabs( adjusted_fx ) < threshold_x ) ? 0.0f : adjusted_fx;
    out_ay = ( std::fabs( adjusted_fy ) < threshold_y ) ? 0.0f : adjusted_fy;
    out_az = ( std::fabs( fz ) < threshold_z ) ? 0.0f : fz;
}