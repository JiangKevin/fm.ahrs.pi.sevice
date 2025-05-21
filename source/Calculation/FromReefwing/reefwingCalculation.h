#pragma once
//
#include "Calculation/sensor_db.h"
#include "Filtering/Reefwing/ReefwingAHRS.h"
//
static bool cp_SENSORDB_2_SensorData( SENSOR_DB& sensor_db, SensorData& sd )
{
    sd.aTimeStamp = sensor_db.time;
    sd.gTimeStamp = sensor_db.time;
    sd.mTimeStamp = sensor_db.time;
    sd.ax         = sensor_db.acc_x;
    sd.ay         = sensor_db.acc_y;
    sd.az         = sensor_db.acc_z;
    sd.gx         = sensor_db.gyro_x;
    sd.gy         = sensor_db.gyro_y;
    sd.gz         = sensor_db.gyro_z;
    sd.mx         = sensor_db.mag_x;
    sd.my         = sensor_db.mag_y;
    sd.mz         = sensor_db.mag_z;
    //
    return true;
};
//
static bool cp_SensorData_2_SENSORDB( SensorData& sd, SENSOR_DB& sensor_db )
{
    sensor_db.time   = sd.aTimeStamp;
    sensor_db.time   = sd.gTimeStamp;
    sensor_db.time   = sd.mTimeStamp;
    sensor_db.acc_x  = sd.ax;
    sensor_db.acc_y  = sd.ay;
    sensor_db.acc_z  = sd.az;
    sensor_db.gyro_x = sd.gx;
    sensor_db.gyro_y = sd.gy;
    sensor_db.gyro_z = sd.gz;
    sensor_db.mag_x  = sd.mx;
    sensor_db.mag_y  = sd.my;
    sensor_db.mag_z  = sd.mz;
    //
    return true;
};