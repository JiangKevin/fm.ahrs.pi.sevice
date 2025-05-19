//
void AhrsCalculation::initEkf()
{
    // ---------------------
    // 1. 初始化滤波器状态
    // ---------------------
    // 假设初始位置与速度均为 0
    kf_filter.vecX().setZero();
    // 初始化状态协方差矩阵，根据先验知识设置一个合适的初始值
    kf_filter.matP().setIdentity();
    kf_filter.matP() *= 0.1F;
    // 过程噪声描述系统动态模型中的不确定性，
    // 这里采用单位阵乘以一个小系数，实际值需根据系统实际情况调节
    processNoise = kf::Matrix< DIM_X, DIM_X >::Identity() * 0.04F;
};
//
void AhrsCalculation::runEkf( Eigen::Vector3f& netAcc, float dt, float& vx, float& vy, float& vz, float& px, float& py, float& pz )
{
    // 为每个轴设置不同的阈值，例如：
    // x 轴阈值 0.05, y 轴阈值 0.05, z 轴阈值 0.03 (可以根据实验数据调节)
    Eigen::Vector3f axesThreshold( 0.05f, 0.05f, 0.03f );

    if ( ! isStationary( netAcc, axesThreshold ) )
    {
        // 1. 构造状态转移矩阵 F (6×6)
        kf::Matrix< DIM_X, DIM_X > F = kf::Matrix< DIM_X, DIM_X >::Identity();
        // 利用运动学模型：位置更新时积分速度
        F( 0, 3 ) = dt;
        F( 1, 4 ) = dt;
        F( 2, 5 ) = dt;

        // 2. 构造控制输入矩阵 B (6×3)
        kf::Matrix< DIM_X, 3 > B;
        B.setZero();
        // 位置部分：加速度对位置的贡献为 0.5 * dt²
        B( 0, 0 ) = 0.5F * dt * dt;
        B( 1, 1 ) = 0.5F * dt * dt;
        B( 2, 2 ) = 0.5F * dt * dt;
        // 速度部分：加速度对速度的贡献为 dt
        B( 3, 0 ) = dt;
        B( 4, 1 ) = dt;
        B( 5, 2 ) = dt;

        // 3. 利用运动学公式更新状态： x_new = F * x_old + B * netAcc
        kf_filter.vecX() = F * kf_filter.vecX() + B * netAcc;

        // 4. 更新状态协方差矩阵： P = F * P * F^T + Q
        kf_filter.matP() = F * kf_filter.matP() * F.transpose() + processNoise;
        //
        px = kf_filter.vecX()[ 0 ];
        py = kf_filter.vecX()[ 1 ];
        pz = kf_filter.vecX()[ 2 ];
        vx = kf_filter.vecX()[ 3 ];
        vy = kf_filter.vecX()[ 4 ];
        vz = kf_filter.vecX()[ 5 ];
    }
    else
    {
        vx = 0.0f;
        vy = 0.0f;
        vz = 0.0f;
    }
};
