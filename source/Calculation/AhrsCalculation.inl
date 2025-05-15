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
    static LowPassFilter filterX( 0.1f );
    static LowPassFilter filterY( 0.1f );
    static LowPassFilter filterZ( 0.1f );

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