#include "AhrsCalculation.h"
#include "Calculation/comput.h"
#include <cstdio>
#include <time.h>
//
// 带 Context* 参数的构造函数实现
AhrsCalculation::AhrsCalculation()
{
    ResetInitFusion();
}

//
bool AhrsCalculation::SolveAnCalculation( SENSOR_DB* sensor_data, SENSOR_DB* original_sensor_data )
{
    float elapsed_time = ( float )( getMicrosecondTimestamp() - start_time ) / ( float )CLOCKS_PER_SEC;
    // Acquire latest sensor data
    const int64_t timestamp = sensor_data->time;
    // printf( "Timestamp Delta Time: %ld\n", timestamp );

    FusionVector gyroscope     = { sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z };
    FusionVector accelerometer = { sensor_data->acc_x, sensor_data->acc_y, sensor_data->acc_z };
    FusionVector magnetometer  = { sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z };

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
    sensor_data->deltaTime = deltaTime;
    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    auto              quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler euler = FusionQuaternionToEuler( quate );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
    // const FusionVector earth = FusionAhrsGetLinearAcceleration( &ahrs );
    //
    original_sensor_data->quate_x = quate.element.x;
    original_sensor_data->quate_y = quate.element.y;
    original_sensor_data->quate_z = quate.element.z;
    original_sensor_data->quate_w = quate.element.w;
    //
    original_sensor_data->roll  = euler.angle.roll;
    original_sensor_data->pitch = euler.angle.pitch;
    original_sensor_data->yaw   = euler.angle.yaw;
    //
    original_sensor_data->eacc_x = earth.axis.x;
    original_sensor_data->eacc_y = earth.axis.y;
    original_sensor_data->eacc_z = earth.axis.z;
    //
    if ( ! CalculateVelAndPos( original_sensor_data, deltaTime, false ) )
    {
        return false;
    }
    //
    sensor_data->quate_x = quate.element.x;
    sensor_data->quate_y = quate.element.y;
    sensor_data->quate_z = quate.element.z;
    sensor_data->quate_w = quate.element.w;
    //
    sensor_data->roll  = euler.angle.roll;
    sensor_data->pitch = euler.angle.pitch;
    sensor_data->yaw   = euler.angle.yaw;
    //
    sensor_data->eacc_x = earth.axis.x;
    sensor_data->eacc_y = earth.axis.y;
    sensor_data->eacc_z = earth.axis.z;
    //
    if ( ! CalculateVelAndPos( sensor_data, deltaTime, true ) )
    {
        return false;
    }
    //
    return true;
}
//
bool AhrsCalculation::CalculateVelAndPos( SENSOR_DB* sensor_data, float dt, bool is_gd )
{

    //
    if ( ! previousAcceleration_init )
    {
        previousAcceleration.resize( 3 );
        previousAcceleration << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
        previousAcceleration_init = true;
    }
    //
    if ( is_gd )
    {
        Eigen::Vector3f acc( sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z );

        // 为每个轴设置不同的阈值
        Eigen::Vector3f axesThreshold( 0.03f, 0.05f, 0.2f );
        //
        auto is_quiescence = isStationary( acc, axesThreshold );
        printf( "is_quiescence: %d , dalta_index: %d \n", is_quiescence, dalta_index );
        //
        if ( ! is_quiescence )
        {
            dalta_index = 0;
            //
            //
            Eigen::VectorXf a_next = Eigen::VectorXf::Zero( 3 );
            a_next << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
            auto ret_v = computeVelocityOfTrapezoid( dt, previousAcceleration, a_next );
            //
            previousAcceleration << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
            //
            original_initialVelocity.axis.x += ret_v[ 0 ];
            original_initialVelocity.axis.y += ret_v[ 1 ];
            original_initialVelocity.axis.z += ret_v[ 2 ];
            original_initialPosition.axis.x = original_initialPosition.axis.x + ( original_initialVelocity.axis.x * dt );
            original_initialPosition.axis.y = original_initialPosition.axis.y + ( original_initialVelocity.axis.y * dt );
            original_initialPosition.axis.z = original_initialPosition.axis.z + ( original_initialVelocity.axis.z * dt );
            //
            sensor_data->vel_x = original_initialVelocity.axis.x;
            sensor_data->vel_y = original_initialVelocity.axis.y;
            sensor_data->vel_z = original_initialVelocity.axis.z;
            //
            sensor_data->pos_x = original_initialPosition.axis.x;
            sensor_data->pos_y = original_initialPosition.axis.y;
            sensor_data->pos_z = original_initialPosition.axis.z;
        }
        else
        {
            dalta_index++;
            //
            if ( dalta_index > 10 )
            {
                dalta_index = 11;
                //
                sensor_data->eacc_x = 0.0f;
                sensor_data->eacc_y = 0.0f;
                sensor_data->eacc_z = 0.0f;
                //
                sensor_data->vel_x = 0.0f;
                sensor_data->vel_y = 0.0f;
                sensor_data->vel_z = 0.0f;
                //
                sensor_data->pos_x = initialPosition.axis.x;
                sensor_data->pos_y = initialPosition.axis.y;
                sensor_data->pos_z = initialPosition.axis.z;
            }
            else
            {
                //
                Eigen::VectorXf a_next = Eigen::VectorXf::Zero( 3 );
                a_next << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
                auto ret_v = computeVelocityOfTrapezoid( dt, previousAcceleration, a_next );
                //
                previousAcceleration << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
                //
                original_initialVelocity.axis.x += ret_v[ 0 ];
                original_initialVelocity.axis.y += ret_v[ 1 ];
                original_initialVelocity.axis.z += ret_v[ 2 ];
                original_initialPosition.axis.x = original_initialPosition.axis.x + ( original_initialVelocity.axis.x * dt );
                original_initialPosition.axis.y = original_initialPosition.axis.y + ( original_initialVelocity.axis.y * dt );
                original_initialPosition.axis.z = original_initialPosition.axis.z + ( original_initialVelocity.axis.z * dt );
                //
                sensor_data->vel_x = original_initialVelocity.axis.x;
                sensor_data->vel_y = original_initialVelocity.axis.y;
                sensor_data->vel_z = original_initialVelocity.axis.z;
                //
                sensor_data->pos_x = original_initialPosition.axis.x;
                sensor_data->pos_y = original_initialPosition.axis.y;
                sensor_data->pos_z = original_initialPosition.axis.z;
            }
        }
    }
    else
    {
        //
        Eigen::VectorXf a_next = Eigen::VectorXf::Zero( 3 );
        a_next << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
        auto ret_v = computeVelocityOfTrapezoid( dt, previousAcceleration, a_next );
        //
        previousAcceleration << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
        //
        original_initialVelocity.axis.x += ret_v[ 0 ];
        original_initialVelocity.axis.y += ret_v[ 1 ];
        original_initialVelocity.axis.z += ret_v[ 2 ];
        
        original_initialPosition.axis.x = original_initialPosition.axis.x + ( original_initialVelocity.axis.x * dt );
        original_initialPosition.axis.y = original_initialPosition.axis.y + ( original_initialVelocity.axis.y * dt );
        original_initialPosition.axis.z = original_initialPosition.axis.z + ( original_initialVelocity.axis.z * dt );
        //
        sensor_data->vel_x = original_initialVelocity.axis.x;
        sensor_data->vel_y = original_initialVelocity.axis.y;
        sensor_data->vel_z = original_initialVelocity.axis.z;
        //
        sensor_data->pos_x = original_initialPosition.axis.x;
        sensor_data->pos_y = original_initialPosition.axis.y;
        sensor_data->pos_z = original_initialPosition.axis.z;
    }

    //
    return true;
}
//
void AhrsCalculation::ResetInitial()
{
    initialVelocity.axis.x = 0.0f;
    initialVelocity.axis.y = 0.0f;
    initialVelocity.axis.z = 0.0f;
    //
    initialPosition.axis.x = 0.0f;
    initialPosition.axis.y = 0.0f;
    initialPosition.axis.z = 0.0f;
    //
    original_initialVelocity.axis.x = 0.0f;
    original_initialVelocity.axis.y = 0.0f;
    original_initialVelocity.axis.z = 0.0f;
    //
    original_initialPosition.axis.x = 0.0f;
    original_initialPosition.axis.y = 0.0f;
    original_initialPosition.axis.z = 0.0f;
    //
    previousTimestamp = getMicrosecondTimestamp();
    //
    previousAcceleration_init = false;
}
//
void AhrsCalculation::ResetInitFusion()
{
    FusionOffsetInitialise( &offset, SAMPLE_RATE );
    FusionAhrsInitialise( &ahrs );
    //
    FusionAhrsSetSettings( &ahrs, &settings );
    //
    previousTimestamp = getMicrosecondTimestamp();
    //
    previousAcceleration_init = false;
}
//
void AhrsCalculation::ConfigFusion( std::string content )
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
std::string AhrsCalculation::GetConfigString()
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
#include "AhrsCalculation.inl"