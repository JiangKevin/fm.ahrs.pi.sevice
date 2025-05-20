#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

using Eigen::VectorXf;
using std::cout;
using std::endl;
using std::mt19937;
using std::normal_distribution;
using std::random_device;
using std::vector;

// 梯度下降优化器
class GradientDescent
{
private:
    double learningRate;
    int    maxIterations;
    double tolerance;
    bool   hasConverged;
public:
    GradientDescent( double lr = 0.01, int maxIter = 1000, double tol = 1e-6 ) : learningRate( lr ), maxIterations( maxIter ), tolerance( tol ), hasConverged( false ) {}

    // 使用数值微分计算梯度
    VectorXf computeGradient( const std::function< double( const VectorXf& ) >& costFunction, const VectorXf& params )
    {

        VectorXf gradient = VectorXf::Zero( params.size() );
        double   h        = 1e-8;

        for ( int i = 0; i < params.size(); ++i )
        {
            VectorXf paramsPlusH = params;
            paramsPlusH( i ) += h;

            double costPlus  = costFunction( paramsPlusH );
            double costMinus = costFunction( params );

            gradient( i ) = ( costPlus - costMinus ) / h;
        }

        return gradient;
    }

    // 执行梯度下降
    VectorXf minimize( const std::function< double( const VectorXf& ) >& costFunction, const VectorXf& initialParams )
    {
        VectorXf params = initialParams;
        int      iter   = 0;
        hasConverged    = false;

        while ( iter < maxIterations )
        {
            VectorXf gradient = computeGradient( costFunction, params );
            double   gradNorm = gradient.norm();

            // 检查收敛条件
            if ( gradNorm < tolerance )
            {
                hasConverged = true;
                break;
            }

            // 更新参数
            params -= learningRate * gradient;

            iter++;
        }

        return params;
    }

    bool converged() const
    {
        return hasConverged;
    }
};

// 加速度计校准模型
class AccelerometerCalibration
{
private:
    // 校准参数: 偏置(3x1)和尺度因子(3x1)
    VectorXf calibrationParams;

    // 模拟生成有噪声的加速度数据
    vector< VectorXf > generateSimulatedData( const VectorXf& trueAccel, const VectorXf& bias, const VectorXf& scale, int numSamples, double noiseLevel )
    {
        vector< VectorXf >            data;
        random_device                 rd;
        mt19937                       gen( rd() );
        normal_distribution< double > noise( 0.0, noiseLevel );

        for ( int i = 0; i < numSamples; ++i )
        {
            // 应用偏置和尺度因子，并添加噪声
            VectorXf noisyAccel = ( trueAccel + bias ).cwiseProduct( scale );
            for ( int j = 0; j < noisyAccel.size(); ++j )
            {
                noisyAccel( j ) += noise( gen );
            }
            data.push_back( noisyAccel );
        }

        return data;
    }
public:
    AccelerometerCalibration() : calibrationParams( 6 )
    {
        // 初始校准参数: [0, 0, 0, 1, 1, 1]
        calibrationParams.setZero();
        calibrationParams( 3 ) = 1.0;
        calibrationParams( 4 ) = 1.0;
        calibrationParams( 5 ) = 1.0;
    }

    // 使用梯度下降校准加速度计
    void calibrate( const VectorXf& expectedGravity )
    {
        // 生成模拟数据
        VectorXf trueBias( 3 );
        trueBias << 0.1, -0.2, 0.15;  // 真实偏置

        VectorXf trueScale( 3 );
        trueScale << 1.05, 0.98, 1.02;  // 真实尺度因子

        vector< VectorXf > samples = generateSimulatedData( expectedGravity, trueBias, trueScale, 1000, 0.01 );

        // 定义代价函数: 校准后数据与期望重力的均方误差
        auto costFunction = [ & ]( const VectorXf& params )
        {
            double totalError = 0.0;

            for ( const auto& sample : samples )
            {
                // 提取偏置和尺度因子
                VectorXf bias  = params.head( 3 );
                VectorXf scale = params.tail( 3 );

                // 应用校准参数
                VectorXf calibrated = ( sample - bias ).cwiseQuotient( scale );

                // 计算与期望重力的误差
                VectorXf error = calibrated - expectedGravity;
                totalError += error.squaredNorm();
            }

            return totalError / samples.size();
        };

        // 使用梯度下降最小化代价函数
        GradientDescent optimizer( 0.01, 1000, 1e-6 );
        VectorXf        initialGuess = calibrationParams;
        calibrationParams            = optimizer.minimize( costFunction, initialGuess );

        cout << "校准结果:" << endl;
        cout << "偏置:" << endl << calibrationParams.head( 3 ) << endl;
        cout << "尺度因子:" << endl << calibrationParams.tail( 3 ) << endl;
        cout << "是否收敛: " << ( optimizer.converged() ? "是" : "否" ) << endl;
    }

    // 应用校准参数到原始加速度数据
    VectorXf applyCalibration( const VectorXf& rawAccel ) const
    {
        VectorXf bias  = calibrationParams.head( 3 );
        VectorXf scale = calibrationParams.tail( 3 );
        return ( rawAccel - bias ).cwiseQuotient( scale );
    }
};
