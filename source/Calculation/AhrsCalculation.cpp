#include "AhrsCalculation.h"
#include "comput.h"
#include <cstdio>
#include <time.h>
//
// 带 Context* 参数的构造函数实现
AhrsCalculation::AhrsCalculation()
{
    ResetInitFusion();
    //
    initKF();
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
    auto               quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler  euler = FusionQuaternionToEuler( quate );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
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
    if ( ! calculateSurfaceVelocity( original_sensor_data, deltaTime, false ) )
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
    // UseKF( sensor_data, deltaTime );
    //
    if ( ! calculateSurfaceVelocity( sensor_data, deltaTime, true ) )
    {
        return false;
    }
    //
    return true;
}
//
bool AhrsCalculation::calculateSurfaceVelocity( SENSOR_DB* sensor_data, float dt, bool is_lp )
{

    float out_x = 0, out_y = 0, out_z = 0;
    if ( is_lp )
    {
        // filterAcceleration( sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z, out_x, out_y, out_z, 0.025f, 0.025f, 0.025f );
        // filterAccelerationWithAngles( sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z, sensor_data->roll, sensor_data->pitch, sensor_data->yaw, out_x, out_y, out_z, 0.025f, 0.025f, 0.025f );
        // filterAccelerationWithMagnetometer( sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z, sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z, out_x, out_y, out_z, 0.025f, 0.025f, 0.025f );
        filterAccelerationWithYawNoGravity( sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z, sensor_data->yaw, out_x, out_y, out_z, 0.025f, 0.025f, 0.025f );

        //
        sensor_data->eacc_x = out_x;
        sensor_data->eacc_y = out_y;
        sensor_data->eacc_z = out_z;
    }
    //
    if ( ! previousAcceleration_init )
    {
        //
        previousAcceleration.axis.x = sensor_data->eacc_x;
        previousAcceleration.axis.y = sensor_data->eacc_y;
        previousAcceleration.axis.z = sensor_data->eacc_z;
        //
        previousAcceleration_init = true;
        return false;
    }
    // printf( "out_X= %f ,out_y= %f ,out_X= %f\n", out_x, out_y, out_z );
    if ( ( out_x == 0 ) && ( out_y == 0 ) && ( out_z == 0 ) )
    {
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
        Eigen::VectorXf a_next( 3 );
        a_next << sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z;
        // std::cout << "dt:" << dt << " a_next:\t" << a_next.transpose() << "\n";

        Eigen::VectorXf a_prev( 3 );
        a_prev << previousAcceleration.axis.x, previousAcceleration.axis.y, previousAcceleration.axis.z;
        // std::cout << "dt:" << dt << " a_prev:\t" << a_prev.transpose() << "\n";

        auto cur_velocity = computeVelocityOfTrapezoid( dt, a_prev, a_next );
        // std::cout << "dt:" << dt << " cur_velocity:\t" << cur_velocity.transpose() << "\n";

        // Calculate the velocity using the trapezoidal rule
        if ( out_x != 0.0f )
        {
            initialVelocity.axis.x = initialVelocity.axis.x + ( cur_velocity[ 0 ] );
        }
        else
        {
            initialVelocity.axis.x = 0.0f;
        }
        if ( out_y != 0.0f )
        {
            initialVelocity.axis.y = initialVelocity.axis.y + ( cur_velocity[ 1 ] );
        }
        else
        {
            initialVelocity.axis.y = 0.0f;
        }
        if ( out_z != 0.0f )
        {
            initialVelocity.axis.z = initialVelocity.axis.z + ( cur_velocity[ 2 ] );
        }
        else
        {
            initialVelocity.axis.z = 0.0f;
        }
        //
        sensor_data->vel_x = initialVelocity.axis.x;
        sensor_data->vel_y = initialVelocity.axis.y;
        sensor_data->vel_z = initialVelocity.axis.z;

        // Calculate the position using the trapezoidal rule
        if ( sensor_data->vel_x != 0.0f )
        {
            initialPosition.axis.x = initialPosition.axis.x + ( sensor_data->vel_x * dt );
        }
        if ( sensor_data->vel_y != 0.0f )
        {
            initialPosition.axis.y = initialPosition.axis.y + ( sensor_data->vel_y * dt );
        }
        if ( sensor_data->vel_z != 0.0f )
        {
            initialPosition.axis.z = initialPosition.axis.z + ( sensor_data->vel_z * dt );
        }
        //
        sensor_data->pos_x = initialPosition.axis.x;
        sensor_data->pos_y = initialPosition.axis.y;
        sensor_data->pos_z = initialPosition.axis.z;
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
    previousAcceleration.axis.x = 0.0f;
    previousAcceleration.axis.y = 0.0f;
    previousAcceleration.axis.z = 0.0f;
    //
    previousTimestamp = getMicrosecondTimestamp();
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

    previousAcceleration.axis.x = 0.0f;
    previousAcceleration.axis.y = 0.0f;
    previousAcceleration.axis.z = 0.0f;
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