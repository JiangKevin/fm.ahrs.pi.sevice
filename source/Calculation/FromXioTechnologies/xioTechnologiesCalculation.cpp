#include "Calculation/comput.h"
#include "xioTechnologiesCalculation.h"
#include <cstdio>
#include <time.h>
//
// 带 Context* 参数的构造函数实现
xioTechnologiesCalculation::xioTechnologiesCalculation()
{
    ResetInitFusion();
}
//
bool xioTechnologiesCalculation::Mul_SolveAnCalculation( EIGEN_SENSOR_DATA* sensor_data, EIGEN_SENSOR_DATA* original_sensor_data )
{
    float elapsed_time = ( float )( getMicrosecondTimestamp() - start_time ) / ( float )CLOCKS_PER_SEC;
    // Acquire latest sensor data
    const int64_t timestamp = sensor_data->time;
    // printf( "Timestamp Delta Time: %ld\n", timestamp );

    FusionVector gyroscope     = { sensor_data->gyr[ 0 ], sensor_data->gyr[ 1 ], sensor_data->gyr[ 2 ] };
    FusionVector accelerometer = { sensor_data->acc[ 0 ], sensor_data->acc[ 1 ], sensor_data->acc[ 2 ] };
    FusionVector magnetometer  = { sensor_data->mag[ 0 ], sensor_data->mag[ 1 ], sensor_data->mag[ 2 ] };

    // Apply calibration
    gyroscope     = FusionCalibrationInertial( gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset );
    accelerometer = FusionCalibrationInertial( accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset );
    magnetometer  = FusionCalibrationMagnetic( magnetometer, softIronMatrix, hardIronOffset );

    // printf("Offset")
    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate( &offset, gyroscope );

    // Calculate delta time (in seconds) to account for gyroscope sample clock error

    deltaTime         = ( float )( timestamp - previousTimestamp ) / ( float )CLOCKS_PER_SEC;
    previousTimestamp = timestamp;
    //
    // printf( "Delta Time: %f\n", deltaTime );
    //
    if ( deltaTime > 1.0 )
    {
        return false;
    }
    //
    original_sensor_data->deltaTime = deltaTime;
    sensor_data->deltaTime          = deltaTime;
    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    auto               quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler  euler = FusionQuaternionToEuler( quate );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
    // const FusionVector earth = FusionAhrsGetLinearAcceleration( &ahrs );
    //
    original_sensor_data->qua[ 0 ] = quate.element.x;
    original_sensor_data->qua[ 1 ] = quate.element.y;
    original_sensor_data->qua[ 2 ] = quate.element.z;
    original_sensor_data->qua[ 3 ] = quate.element.w;
    //
    original_sensor_data->eul[ 0 ] = euler.angle.roll;
    original_sensor_data->eul[ 1 ] = euler.angle.pitch;
    original_sensor_data->eul[ 2 ] = euler.angle.yaw;
    //
    original_sensor_data->eacc[ 0 ] = earth.axis.x;
    original_sensor_data->eacc[ 1 ] = earth.axis.y;
    original_sensor_data->eacc[ 2 ] = earth.axis.z;
    original_sensor_data->totalAcc  = original_sensor_data->eacc.norm();

    //
    if ( ! Mul_CalculateVelAndPos( original_sensor_data, deltaTime, false ) )
    {
        return false;
    }
    //
    sensor_data->qua[ 0 ] = quate.element.x;
    sensor_data->qua[ 1 ] = quate.element.y;
    sensor_data->qua[ 2 ] = quate.element.z;
    sensor_data->qua[ 3 ] = quate.element.w;
    //
    sensor_data->eul[ 0 ] = euler.angle.roll;
    sensor_data->eul[ 1 ] = euler.angle.pitch;
    sensor_data->eul[ 2 ] = euler.angle.yaw;
    //
    sensor_data->eacc[ 0 ] = earth.axis.x;
    sensor_data->eacc[ 1 ] = earth.axis.y;
    sensor_data->eacc[ 2 ] = earth.axis.z;
    sensor_data->totalAcc  = sensor_data->eacc.norm();

    //
    if ( ! Mul_CalculateVelAndPos( sensor_data, deltaTime, true ) )
    {
        return false;
    }
    //
    return true;
}
//
bool xioTechnologiesCalculation::Mul_CalculateVelAndPos( EIGEN_SENSOR_DATA* sensor_data, float dt, bool is_hp )
{
    // 为每个轴设置不同的阈值
    float           axesThreshold_x = 0.0f, axesThreshold_y = 0.0f, axesThreshold_z = 0.0f;
    Eigen::VectorXf mul_previousAcceleration = Eigen::VectorXf::Zero( 3 );
    //
    if ( is_hp )
    {
        axesThreshold_x = 0.01f;
        axesThreshold_y = 0.02f;
        axesThreshold_z = 0.01f;
    }
    else
    {
        axesThreshold_x = 0.0f;
        axesThreshold_y = 0.0f;
        axesThreshold_z = 0.0f;
    }
    //
    if ( ! mul_previousAcceleration_init )
    {
        isStationary( sensor_data, axesThreshold_x, axesThreshold_y, axesThreshold_z );
        mul_previousAcceleration << sensor_data->eacc[ 0 ], sensor_data->eacc[ 1 ], sensor_data->eacc[ 2 ];
        mul_previousAcceleration_init = true;
        //
        return false;
    }
    //
    isStationary( sensor_data, axesThreshold_x, axesThreshold_y, axesThreshold_z );
    //
    Eigen::VectorXf a_next = Eigen::VectorXf::Zero( 3 );
    a_next << sensor_data->eacc[ 0 ], sensor_data->eacc[ 1 ], sensor_data->eacc[ 2 ];
    auto ret_v = computeVelocityOfTrapezoid( dt, mul_previousAcceleration, a_next );
    //
    mul_previousAcceleration << sensor_data->eacc[ 0 ], sensor_data->eacc[ 1 ], sensor_data->eacc[ 2 ];
    //
    sensor_data->vel[ 0 ] = ret_v[ 0 ];
    sensor_data->vel[ 1 ] = ret_v[ 1 ];
    sensor_data->vel[ 2 ] = ret_v[ 2 ];

    sensor_data->pos[ 0 ] += ( sensor_data->vel[ 0 ] * dt );
    sensor_data->pos[ 1 ] += ( sensor_data->vel[ 1 ] * dt );
    sensor_data->pos[ 2 ] += ( sensor_data->vel[ 2 ] * dt );

    //
    return true;
}
//
void xioTechnologiesCalculation::ResetInitial()
{
    //
    previousTimestamp = getMicrosecondTimestamp();
    //
    mul_previousAcceleration_init = false;
    one_previousAcceleration_init = false;
}
//
void xioTechnologiesCalculation::ResetInitFusion()
{
    FusionOffsetInitialise( &offset, SAMPLE_RATE );
    FusionAhrsInitialise( &ahrs );
    //
    FusionAhrsSetSettings( &ahrs, &settings );
    //
    previousTimestamp = getMicrosecondTimestamp();
    //
    mul_previousAcceleration_init = false;
    one_previousAcceleration_init = false;
}
//
void xioTechnologiesCalculation::ConfigFusion( std::string content )
{
    char delimiter = ',';
    auto values    = splitString( content, delimiter );
    //
    if ( values.size() == 49 )
    {
        gyroscopeMisalignment.element.xx = std::stof( values[ 1 ] );
        gyroscopeMisalignment.element.xy = std::stof( values[ 2 ] );
        gyroscopeMisalignment.element.xz = std::stof( values[ 3 ] );
        gyroscopeMisalignment.element.yx = std::stof( values[ 4 ] );
        gyroscopeMisalignment.element.yy = std::stof( values[ 5 ] );
        gyroscopeMisalignment.element.yz = std::stof( values[ 6 ] );
        gyroscopeMisalignment.element.zx = std::stof( values[ 7 ] );
        gyroscopeMisalignment.element.zy = std::stof( values[ 8 ] );
        gyroscopeMisalignment.element.zz = std::stof( values[ 9 ] );
        gyroscopeSensitivity.axis.x      = std::stof( values[ 10 ] );
        gyroscopeSensitivity.axis.y      = std::stof( values[ 11 ] );
        gyroscopeSensitivity.axis.z      = std::stof( values[ 12 ] );
        gyroscopeOffset.axis.x           = std::stof( values[ 13 ] );
        gyroscopeOffset.axis.y           = std::stof( values[ 14 ] );
        gyroscopeOffset.axis.z           = std::stof( values[ 15 ] );
        //
        gyroscopeMisalignment.element.xx = std::stof( values[ 16 ] );
        gyroscopeMisalignment.element.xy = std::stof( values[ 17 ] );
        gyroscopeMisalignment.element.xz = std::stof( values[ 18 ] );
        gyroscopeMisalignment.element.yx = std::stof( values[ 19 ] );
        gyroscopeMisalignment.element.yx = std::stof( values[ 20 ] );
        gyroscopeMisalignment.element.yx = std::stof( values[ 21 ] );
        gyroscopeMisalignment.element.zx = std::stof( values[ 22 ] );
        gyroscopeMisalignment.element.zx = std::stof( values[ 23 ] );
        gyroscopeMisalignment.element.zx = std::stof( values[ 24 ] );
        accelerometerSensitivity.axis.x  = std::stof( values[ 25 ] );
        accelerometerSensitivity.axis.y  = std::stof( values[ 26 ] );
        accelerometerSensitivity.axis.z  = std::stof( values[ 27 ] );
        accelerometerOffset.axis.x       = std::stof( values[ 28 ] );
        accelerometerOffset.axis.y       = std::stof( values[ 29 ] );
        accelerometerOffset.axis.z       = std::stof( values[ 30 ] );
        //
        softIronMatrix.element.xx = std::stof( values[ 31 ] );
        softIronMatrix.element.xy = std::stof( values[ 32 ] );
        softIronMatrix.element.xz = std::stof( values[ 33 ] );
        softIronMatrix.element.yx = std::stof( values[ 34 ] );
        softIronMatrix.element.yy = std::stof( values[ 35 ] );
        softIronMatrix.element.yz = std::stof( values[ 36 ] );
        softIronMatrix.element.zx = std::stof( values[ 37 ] );
        softIronMatrix.element.zy = std::stof( values[ 38 ] );
        softIronMatrix.element.zz = std::stof( values[ 39 ] );
        hardIronOffset.axis.x     = std::stof( values[ 40 ] );
        hardIronOffset.axis.y     = std::stof( values[ 41 ] );
        hardIronOffset.axis.z     = std::stof( values[ 42 ] );
        //
        if ( std::stoi( values[ 43 ] ) == 0 )
        {
            settings.convention = FusionConventionNwu;
        }
        else if ( std::stoi( values[ 43 ] ) == 1 )
        {
            settings.convention = FusionConventionEnu;
        }
        else if ( std::stoi( values[ 43 ] ) == 2 )
        {
            settings.convention = FusionConventionNed;
        }
        settings.gain                  = std::stof( values[ 44 ] );
        settings.gyroscopeRange        = std::stof( values[ 45 ] );
        settings.accelerationRejection = std::stof( values[ 46 ] );
        settings.magneticRejection     = std::stof( values[ 47 ] );
        settings.recoveryTriggerPeriod = std::stoul( values[ 48 ] );
    }
}
//
std::string xioTechnologiesCalculation::GetConfigString()
{
    std::string content_str = "Setup";
    //
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.xx );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.xy );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.xz );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.yx );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.yy );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.yz );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.zx );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.zy );
    content_str += "," + transaction_to_string( gyroscopeMisalignment.element.zz );
    content_str += "," + transaction_to_string( gyroscopeSensitivity.axis.x );
    content_str += "," + transaction_to_string( gyroscopeSensitivity.axis.y );
    content_str += "," + transaction_to_string( gyroscopeSensitivity.axis.z );
    content_str += "," + transaction_to_string( gyroscopeOffset.axis.x );
    content_str += "," + transaction_to_string( gyroscopeOffset.axis.y );
    content_str += "," + transaction_to_string( gyroscopeOffset.axis.z );
    //
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.xx );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.xy );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.xz );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.yx );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.yy );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.yz );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.zx );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.zy );
    content_str += "," + transaction_to_string( accelerometerMisalignment.element.zz );
    content_str += "," + transaction_to_string( accelerometerSensitivity.axis.x );
    content_str += "," + transaction_to_string( accelerometerSensitivity.axis.y );
    content_str += "," + transaction_to_string( accelerometerSensitivity.axis.z );
    content_str += "," + transaction_to_string( accelerometerOffset.axis.x );
    content_str += "," + transaction_to_string( accelerometerOffset.axis.y );
    content_str += "," + transaction_to_string( accelerometerOffset.axis.z );
    //
    content_str += "," + transaction_to_string( softIronMatrix.element.xx );
    content_str += "," + transaction_to_string( softIronMatrix.element.xy );
    content_str += "," + transaction_to_string( softIronMatrix.element.xz );
    content_str += "," + transaction_to_string( softIronMatrix.element.yx );
    content_str += "," + transaction_to_string( softIronMatrix.element.yy );
    content_str += "," + transaction_to_string( softIronMatrix.element.yz );
    content_str += "," + transaction_to_string( softIronMatrix.element.zx );
    content_str += "," + transaction_to_string( softIronMatrix.element.zy );
    content_str += "," + transaction_to_string( softIronMatrix.element.zz );
    content_str += "," + transaction_to_string( hardIronOffset.axis.x );
    content_str += "," + transaction_to_string( hardIronOffset.axis.y );
    content_str += "," + transaction_to_string( hardIronOffset.axis.z );
    //
    content_str += "," + int_transaction_to_string( settings.convention );
    content_str += "," + transaction_to_string( settings.gain );
    content_str += "," + transaction_to_string( settings.gyroscopeRange );
    content_str += "," + transaction_to_string( settings.accelerationRejection );
    content_str += "," + transaction_to_string( settings.magneticRejection );
    content_str += "," + int_transaction_to_string( settings.recoveryTriggerPeriod );
    //
    return content_str;
}

//
#include "xioTechnologiesCalculation.inl"