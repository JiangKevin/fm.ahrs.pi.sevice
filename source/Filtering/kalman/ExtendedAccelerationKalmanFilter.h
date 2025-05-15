#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// 扩展的基于 6 维状态（加速度 + 偏置）的 Kalman 滤波器
class ExtendedAccelerationKalmanFilter
{
public:
    // 空构造函数
    ExtendedAccelerationKalmanFilter() {}

    // 初始化扩展滤波器参数
    // 参数说明：
    //   dt                : 时间步长（如果需要引入积分效应时可用，本例保持不变）
    //   initial_acc       : 初始加速度（3 维向量）
    //   initial_bias      : 初始偏置（3 维向量），可由静态校准获得
    //   process_noise_acc : 加速度部分的过程噪声因子
    //   process_noise_bias: 偏置部分的过程噪声因子（通常较小）
    //   measurement_noise : 观测噪声因子
    void init( float dt, const Eigen::VectorXf& initial_acc, const Eigen::VectorXf& initial_bias, float process_noise_acc, float process_noise_bias, float measurement_noise )
    {
        dt_   = dt;
        int n = 6;  // 状态维度：3 维加速度 + 3 维偏置
        int m = 3;  // 观测维度：3 维测量

        // 初始化状态向量 x = [加速度, 偏置]
        x_.resize( n );
        x_.head( 3 ) = initial_acc;
        x_.tail( 3 ) = initial_bias;

        // 初始化估计误差协方差矩阵为 6x6 单位阵
        P_ = Eigen::MatrixXf::Identity( n, n );

        // 状态转移矩阵 F: 假设加速度与偏置均遵从随机游走，保持不变
        F_ = Eigen::MatrixXf::Identity( n, n );

        // 观测矩阵 H: 观测模型： z = a + b, 所以 H = [I I]
        H_.resize( m, n );
        H_.block( 0, 0, 3, 3 ) = Eigen::MatrixXf::Identity( 3, 3 );
        H_.block( 0, 3, 3, 3 ) = Eigen::MatrixXf::Identity( 3, 3 );

        // 过程噪声 Q: 分别设定加速度与偏置部分的噪声
        Q_                     = Eigen::MatrixXf::Zero( n, n );
        Q_.block( 0, 0, 3, 3 ) = Eigen::MatrixXf::Identity( 3, 3 ) * process_noise_acc;
        Q_.block( 3, 3, 3, 3 ) = Eigen::MatrixXf::Identity( 3, 3 ) * process_noise_bias;

        // 观测噪声 R: 观测噪声协方差矩阵
        R_ = Eigen::MatrixXf::Identity( m, m ) * measurement_noise;
    }

    // 预测步骤
    void predict()
    {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // 更新步骤：采用尺寸为 3 的观测向量 z
    void update( const Eigen::VectorXf& z )
    {
        // 计算观测残差
        Eigen::VectorXf y = z - H_ * x_;
        // 计算残差协方差矩阵
        Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;
        // 计算卡尔曼增益
        Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();

        // 更新状态向量
        x_    = x_ + K * y;
        int n = x_.size();
        // 更新误差协方差矩阵
        P_ = ( Eigen::MatrixXf::Identity( n, n ) - K * H_ ) * P_;
    }

    // 获取当前状态向量 (加速度和偏置)
    Eigen::VectorXf getState() const
    {
        return x_;
    }
private:
    float           dt_;  // 时间步长
    Eigen::VectorXf x_;   // 状态向量（6维：3加速度 + 3偏置）
    Eigen::MatrixXf P_;   // 估计误差协方差矩阵
    Eigen::MatrixXf F_;   // 状态转移矩阵
    Eigen::MatrixXf H_;   // 观测矩阵
    Eigen::MatrixXf Q_;   // 过程噪声协方差矩阵
    Eigen::MatrixXf R_;   // 观测噪声协方差矩阵
};
