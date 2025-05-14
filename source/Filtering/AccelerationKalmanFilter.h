#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include <iostream>

// 针对加速度滤波的标准卡尔曼滤波器类，状态均采用 Eigen::VectorXf 表示
class AccelerationKalmanFilter
{
public:
    // 空构造函数，不做初始化
    AccelerationKalmanFilter() {}

    // 初始化滤波器
    // dt               : 时间步长（示例中未直接使用，但保留用于扩展）
    // initial_acc      : 初始加速度
    // process_noise    : 过程噪声（用于构造过程噪声协方差矩阵 Q）
    // measurement_noise: 观测噪声（用于构造观测噪声协方差矩阵 R）
    void init( float dt, float initial_acc, float process_noise, float measurement_noise )
    {
        dt_   = dt;
        int n = 1;  // 状态维度（此处仅处理一维加速度）
        int m = 1;  // 观测维度

        // 初始化状态向量（初始加速度）
        x_ = Eigen::VectorXf::Constant( n, initial_acc );

        // 初始化误差协方差矩阵为单位阵
        P_ = Eigen::MatrixXf::Identity( n, n );

        // 状态转移矩阵 F：使用随机游走模型，认为加速度不发生系统性变化
        F_ = Eigen::MatrixXf::Identity( n, n );

        // 观测矩阵 H：直接观测加速度，所以 H 为单位矩阵
        H_ = Eigen::MatrixXf::Identity( m, n );

        // 过程噪声协方差 Q：反映状态变化的不确定性
        Q_ = Eigen::MatrixXf::Identity( n, n ) * process_noise;

        // 观测噪声协方差 R：反映观测噪声水平
        R_ = Eigen::MatrixXf::Identity( m, m ) * measurement_noise;
    }

    // 预测步骤：利用状态转移模型预测下一时刻的状态和更新协方差
    void predict()
    {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // 更新步骤：输入观测值 z（1 维向量），计算卡尔曼增益并更新状态
    void update( const Eigen::VectorXf& z )
    {
        Eigen::VectorXf y = z - H_ * x_;                        // 观测残差
        Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;      // 残差协方差矩阵
        Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();  // 卡尔曼增益

        x_ = x_ + K * y;  // 更新状态估计
        P_ = ( Eigen::MatrixXf::Identity( x_.size(), x_.size() ) - K * H_ ) * P_;
    }

    // 获取当前状态估计（加速度）
    Eigen::VectorXf getState() const
    {
        return x_;
    }
private:
    float           dt_;  // 时间步长（扩展时可用）
    Eigen::VectorXf x_;   // 状态向量（1 维：加速度）
    Eigen::MatrixXf P_;   // 误差协方差矩阵
    Eigen::MatrixXf F_;   // 状态转移矩阵
    Eigen::MatrixXf H_;   // 观测矩阵
    Eigen::MatrixXf Q_;   // 过程噪声协方差矩阵
    Eigen::MatrixXf R_;   // 观测噪声协方差矩阵
};