//
void AhrsCalculation::UseKF( SENSOR_DB* sensor_data, float dt )
{
    // 模拟真实加速度（理想数据，无噪声），例如恒定加速度 [1.5, -2.0, 0.5] m/s²
    Eigen::VectorXf true_acc( 3 );
    true_acc << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
    // 1. 预测步骤
    akf.predict();
    Eigen::VectorXf predicted = akf.getState();
    // std::cout << "predicted:\t" << predicted.transpose() << "\n";

    // 2. 测量步骤：这里直接用理想真实加速度
    // std::cout << "true_acc:\t" << true_acc.transpose() << "\n";

    // 3. 更新步骤
    akf.update( true_acc );
    Eigen::VectorXf updated = akf.getState();

    // 输出预测、测量和更新后的加速度（单位：m/s²）
    // std::cout << "updated:\t" << updated.transpose() << "\n";
    //
    Eigen::VectorXf result = updated.transpose();
    //
    sensor_data->eacc_x = result[ 0 ];
    sensor_data->eacc_y = result[ 1 ];
    sensor_data->eacc_z = result[ 2 ];
}

void AhrsCalculation ::initKF()
{
    // 初始化滤波器：
    // 时间步长 dt = 0.065 s；
    // 初始加速度：假设初始为 [0, 0, 0] m/s²；
    // 过程噪声设为 0.001，观测噪声设为 1e-6（理想数据，噪声极小）
    Eigen::VectorXf initial_acc( 3 );
    initial_acc << 0.0f, 0.0f, 0.0f;
    akf.init( 0.065f, initial_acc, 0.001f, 0.0002f );
}