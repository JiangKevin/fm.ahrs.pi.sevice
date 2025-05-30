bool xioTechnologiesCalculation::One_CalculateVelAndPos( EIGEN_SENSOR_DATA* sensor_data, float dt, bool is_hp )
{
    // 为每个轴设置不同的阈值
    float axesThreshold_x = 0.0f, axesThreshold_y = 0.0f, axesThreshold_z = 0.0f;
    //
    if ( is_hp )
    {
        axesThreshold_x = 0.05f;
        axesThreshold_y = 0.2f;
        axesThreshold_z = 0.15f;
    }
    else
    {
        axesThreshold_x = 0.0f;
        axesThreshold_y = 0.0f;
        axesThreshold_z = 0.0f;
    }
    //
    if ( ! one_previousAcceleration_init )
    {
        // one_previousAcceleration.resize( 3 );
        isStationary( sensor_data, axesThreshold_x, axesThreshold_y, axesThreshold_z );
        one_previousAcceleration << sensor_data->eacc[ 0 ], sensor_data->eacc[ 1 ], sensor_data->eacc[ 2 ];
        one_previousAcceleration_init = true;
        //
        return false;
    }
    //
    isStationary( sensor_data, axesThreshold_x, axesThreshold_y, axesThreshold_z );
    //
    Eigen::VectorXf a_next = Eigen::VectorXf::Zero( 3 );
    a_next << sensor_data->eacc[ 0 ], sensor_data->eacc[ 1 ], sensor_data->eacc[ 2 ];
    auto ret_v = computeVelocityOfTrapezoid( dt, one_previousAcceleration, a_next );
    //
    one_previousAcceleration << sensor_data->eacc[ 0 ], sensor_data->eacc[ 1 ], sensor_data->eacc[ 2 ];
    //
    sensor_data->vel[ 0 ] = ret_v[ 0 ];
    sensor_data->vel[ 1 ] = ret_v[ 1 ];
    sensor_data->vel[ 2 ] = ret_v[ 2 ];

    sensor_data->pos[ 0 ] += ( sensor_data->vel[ 0 ] * dt ) * 100000.0f;
    sensor_data->pos[ 1 ] += ( sensor_data->vel[ 1 ] * dt ) * 100000.0f;
    sensor_data->pos[ 2 ] += ( sensor_data->vel[ 2 ] * dt ) * 100000.0f;

    //
    return true;
}
//
//
bool xioTechnologiesCalculation::One_SolveAnCalculation( EIGEN_SENSOR_DATA* sensor_data )
{
    float elapsed_time = ( float )( getMicrosecondTimestamp() - start_time ) / ( float )CLOCKS_PER_SEC;
    // Acquire latest sensor data
    const int64_t timestamp = sensor_data->time;
    //
    FusionVector gyroscope     = { sensor_data->gyr[ 0 ], sensor_data->gyr[ 0 ], sensor_data->gyr[ 0 ] };
    FusionVector accelerometer = { sensor_data->acc[ 0 ], sensor_data->acc[ 1 ], sensor_data->acc[ 2 ] };
    FusionVector magnetometer  = { sensor_data->mag[ 0 ], sensor_data->mag[ 1 ], sensor_data->mag[ 2 ] };

    // Apply calibration
    gyroscope     = FusionCalibrationInertial( gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset );
    accelerometer = FusionCalibrationInertial( accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset );
    magnetometer  = FusionCalibrationMagnetic( magnetometer, softIronMatrix, hardIronOffset );

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate( &offset, gyroscope );

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static float one_deltaTime = ( float )( timestamp - previousTimestamp ) / ( float )CLOCKS_PER_SEC;
    previousTimestamp          = timestamp;
    //
    if ( one_deltaTime > 1.0 )
    {
        return false;
    }
    sensor_data->deltaTime = one_deltaTime;
    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate( &ahrs, gyroscope, accelerometer, magnetometer, deltaTime );

    // Print algorithm outputs
    auto               quate = FusionAhrsGetQuaternion( &ahrs );
    const FusionEuler  euler = FusionQuaternionToEuler( quate );
    const FusionVector earth = FusionAhrsGetEarthAcceleration( &ahrs );
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
    //
    sensor_data->totalAcc = sensor_data->eacc.norm();
    //
    if ( ! One_CalculateVelAndPos( sensor_data, deltaTime, true ) )
    {
        return false;
    }
    //
    return true;
};