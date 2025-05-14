#include <Eigen/Dense>
#include <iostream>

// 针对3维加速度的标准卡尔曼滤波器类，状态向量为尺寸为 3 的 Eigen::VectorXf
class AccelerationKalmanFilter
{
public:
    // 空构造函数，不做初始化
    AccelerationKalmanFilter() {}

    // 初始化滤波器参数
    // 参数说明：
    //   dt                : 时间步长（预留扩展用，本例中无明显影响）
    //   initial_acc       : 初始加速度，尺寸为 3 的 Eigen::VectorXf
    //   process_noise     : 过程噪声协方差因子
    //   measurement_noise : 观测噪声协方差因子（由于数据无噪声，可以取一个很小的值）
    void init( float dt, const Eigen::VectorXf& initial_acc, float process_noise, float measurement_noise )
    {
        dt_   = dt;
        int n = 3;  // 状态维度：3维加速度
        int m = 3;  // 观测维度：3维测量

        // 状态向量：设为初始加速度（3维）
        x_ = initial_acc;

        // 初始化估计误差协方差矩阵为单位阵
        P_ = Eigen::MatrixXf::Identity( n, n );

        // 状态转移矩阵 F：采用随机游走模型，即假设加速度保持不变
        F_ = Eigen::MatrixXf::Identity( n, n );

        // 观测矩阵 H：直接观测加速度，因此 H 为 3×3 单位阵
        H_ = Eigen::MatrixXf::Identity( m, n );

        // 过程噪声协方差矩阵 Q
        Q_ = Eigen::MatrixXf::Identity( n, n ) * process_noise;

        // 观测噪声协方差矩阵 R（数据无噪声，可取很小的值）
        R_ = Eigen::MatrixXf::Identity( m, m ) * measurement_noise;
    }

    // 预测步骤
    void predict()
    {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // 更新步骤：采用尺寸为3的观测向量 z 进行状态更新
    void update( const Eigen::VectorXf& z )
    {
        Eigen::VectorXf y = z - H_ * x_;                        // 计算观测残差
        Eigen::MatrixXf S = H_ * P_ * H_.transpose() + R_;      // 计算残差协方差矩阵
        Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();  // 计算卡尔曼增益

        x_    = x_ + K * y;  // 更新状态向量
        int n = x_.size();
        P_    = ( Eigen::MatrixXf::Identity( n, n ) - K * H_ ) * P_;  // 更新误差协方差矩阵
    }

    // 返回当前状态估计（3维加速度）
    Eigen::VectorXf getState() const
    {
        return x_;
    }
private:
    float           dt_;  // 时间步长（预留扩展用）
    Eigen::VectorXf x_;   // 状态向量（3维加速度）
    Eigen::MatrixXf P_;   // 误差协方差矩阵
    Eigen::MatrixXf F_;   // 状态转移矩阵
    Eigen::MatrixXf H_;   // 观测矩阵
    Eigen::MatrixXf Q_;   // 过程噪声协方差矩阵
    Eigen::MatrixXf R_;   // 观测噪声协方差矩阵
};