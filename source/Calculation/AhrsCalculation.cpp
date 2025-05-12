#include "AhrsCalculation.h"
#include <cstdio>
#include <time.h>
//
// 带 Context* 参数的构造函数实现
AhrsCalculation::AhrsCalculation()
{
    FusionOffsetInitialise( &offset, SAMPLE_RATE );
    FusionAhrsInitialise( &ahrs );
    //
    FusionAhrsSetSettings( &ahrs, &settings );
}

//
void AhrsCalculation::SolveAnCalculation( SENSOR_DB* sensor_data )
{
    // Acquire latest sensor data
    const int64_t timestamp = sensor_data->time;
    // printf( "Timestamp Delta Time: %ld\n", timestamp );

    FusionVector gyroscope     = { sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z };
    FusionVector accelerometer = { sensor_data->acc_x, sensor_data->acc_y, sensor_data->acc_z };
    FusionVector magnetometer  = { sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z };

    // printf( "Gyroscope: %f %f %f\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z );
    // printf( "Accelerometer: %f %f %f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z );
    // printf( "Magnetometer: %f %f %f\n", magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z );
    //
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
        return;
    }
    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    auto               quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler  euler = FusionQuaternionToEuler( quate );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
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
    // printf( "Quaternion: %f %f %f %f\n", sensor_data->quate_x, sensor_data->quate_y, sensor_data->quate_z, sensor_data->quate_w );
    // printf( "Euler: %f %f %f\n", sensor_data->roll, sensor_data->pitch, sensor_data->yaw );
    // printf( "Earth Acceleration: %f %f %f\n", sensor_data->eacc_x, sensor_data->eacc_y, sensor_data->eacc_z );

    //
    calculateSurfaceVelocity( sensor_data, deltaTime );
    //
    // printf( "Estimated Velocity: %f %f %f\n", sensor_data->vel_x, sensor_data->vel_y, sensor_data->vel_z );
    // printf( "Estimated Position: %f %f %f\n", sensor_data->pos_x, sensor_data->pos_y, sensor_data->pos_z );
    //
    // printf( "--------------------------------------------------------------------------------------------------------------------------------------------\n" );
}
//
void AhrsCalculation::calculateSurfaceVelocity( SENSOR_DB* sensor_data, float dt )
{
    // Calculate the velocity using the trapezoidal rule
    initialVelocity.axis.x = initialVelocity.axis.x + ( sensor_data->eacc_x * dt );
    initialVelocity.axis.y = initialVelocity.axis.y + ( sensor_data->eacc_y * dt );
    initialVelocity.axis.z = initialVelocity.axis.z + ( sensor_data->eacc_z * dt );
    //
    sensor_data->vel_x = initialVelocity.axis.x;
    sensor_data->vel_y = initialVelocity.axis.y;
    sensor_data->vel_z = initialVelocity.axis.z;

    //
    // Calculate the position using the trapezoidal rule
    initialPosition.axis.x = initialPosition.axis.x + ( sensor_data->vel_x * dt );
    initialPosition.axis.y = initialPosition.axis.y + ( sensor_data->vel_y * dt );
    initialPosition.axis.z = initialPosition.axis.z + ( sensor_data->vel_z * dt );
    //
    sensor_data->pos_x = initialPosition.axis.x;
    sensor_data->pos_y = initialPosition.axis.y;
    sensor_data->pos_z = initialPosition.axis.z;
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
}
//
void AhrsCalculation::ResetInitFusion()
{
    FusionOffsetInitialise( &offset, SAMPLE_RATE );
    FusionAhrsInitialise( &ahrs );
    //
    FusionAhrsSetSettings( &ahrs, &settings );
}