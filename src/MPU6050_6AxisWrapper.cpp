
#include "MPU6050_6AxisWrapper.h"

namespace arduino_robot
{
bool IMU6050::initialize(uint8_t gyro_sensetivity, uint8_t accel_sensetivity, VectorInt16 gyro_offset, VectorInt16 accel_offset, int16_t g_reference_value)
{
	mpu_.initialize();
	uint8_t dev_status = mpu_.dmpInitialize(gyro_sensetivity, accel_sensetivity);
    
    if (dev_status != 0)
    {
        return false;
    }
	mpu_.setXGyroOffset(gyro_offset.x);
	mpu_.setYGyroOffset(gyro_offset.y);
	mpu_.setZGyroOffset(gyro_offset.z);
	mpu_.setXAccelOffset(accel_offset.x);
	mpu_.setYAccelOffset(accel_offset.y);
	mpu_.setZAccelOffset(accel_offset.z);

	mpu_.setDMPEnabled(true);
	mpu_.setDLPFMode(5);
	mpuIntStatus_ = mpu_.getIntStatus();

	dmpReady_ = true;
	packetSize_ = mpu_.dmpGetFIFOPacketSize();
    
    if (g_reference_value == 0)
    {
        switch (accel_sensetivity)
        {
            case MPU6050_ACCEL_FS_2 : g_reference_value = 8192; break;
            case MPU6050_ACCEL_FS_4 : g_reference_value = 4096; break;
            case MPU6050_ACCEL_FS_8 : g_reference_value = 2048; break;
            case MPU6050_ACCEL_FS_16 : g_reference_value = 1024; break;
        }
    }
	mpu_.g_reference_value = g_reference_value;
    
    return true;
}


void IMU6050::update()
{

	fifoCount_ = mpu_.getFIFOCount();
    
	// wait for mpu_ extra packet(s) available
	while (fifoCount_ < packetSize_) 
    {
		fifoCount_ = mpu_.getFIFOCount();
	}

    mpu_.getFIFOBytes(fifoBuffer, packetSize_);

    //raw values
    mpu_.dmpGetAccel(&aa_, fifoBuffer);
    mpu_.dmpGetGyro(&w_, fifoBuffer);
    mpu_.dmpGetQuaternion(&q_, fifoBuffer);
    
    mpu_.dmpGetGravity(&gravity_, &q_);
    mpu_.dmpGetYawPitchRoll(ypr_, &q_, &gravity_);
    mpu_.dmpGetLinearAccel(&aaReal_, &aa_, &gravity_);
    mpu_.dmpGetLinearAccelInWorld(&aaWorld_, &aaReal_, &q_);
    
    aaWorld_.filterLowValues(accel_low_filter_);
    mpu_.resetFIFO();
}


void IMU6050::getMotion(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) const
{
    ax = aa_.x;
    ay = aa_.y;
    az = aa_.z;
    gx = w_.x;
    gy = w_.y;
    gz = w_.z;
   
    return;
}


void IMU6050::calibrate()
{
    //mpu_.setDMPEnabled(false);
    
    const uint8_t num_calibs = 6;
    typedef void(MPU6050::*calib_fn)(int16_t);
    const int16_t max_calib_value = 8192;
    const uint8_t num_iterations = 20;
    int16_t calib_max[num_calibs] = {max_calib_value, max_calib_value, max_calib_value, max_calib_value, max_calib_value, max_calib_value};
    int16_t calib_min[num_calibs] = {-max_calib_value, -max_calib_value, -max_calib_value, -max_calib_value, -max_calib_value, -max_calib_value};
    long avg_sensor_val[num_calibs];
    int16_t expected_sensor_val[num_calibs] = {0, 0, mpu_.g_reference_value, 0, 0, 0};
   
    calib_fn calib_funcs[num_calibs] = {&MPU6050::setXAccelOffset, &MPU6050::setYAccelOffset, &MPU6050::setZAccelOffset,
                               &MPU6050::setXGyroOffset, &MPU6050::setYGyroOffset, &MPU6050::setZGyroOffset};
                               
    for (uint8_t i = 0; i < num_calibs; i++)
    {
        (mpu_.*calib_funcs[i])(0);
    }
    
    bool calibration_finished = false;
    
    while (!calibration_finished)
    {
        calibration_finished = true;
        //set new calibration
        for (uint8_t i = 0; i < num_calibs; i++)
        {
            int16_t calib_val = (calib_max[i] + calib_min[i])/2;
            (mpu_.*calib_funcs[i])(calib_val);
            avg_sensor_val[i] = 0;
        }
        delay(50);
        update();
        //take avg sensor reading
        for (uint8_t k = 0; k < num_iterations; k++)
        {
            update();
            int16_t sensor_val[num_calibs];
            getMotion(sensor_val[0], sensor_val[1], sensor_val[2], sensor_val[3], sensor_val[4], sensor_val[5]);
            for (uint8_t i = 0; i < num_calibs; i++)
            {
               avg_sensor_val[i] += sensor_val[i];
            }    

        }
        //pick new bound values
        for (uint8_t i = 0; i < num_calibs; i++)
        {
            if (avg_sensor_val[i]/num_iterations > expected_sensor_val[i])
            {
                calib_max[i] = (calib_max[i] + calib_min[i])/2;
            }
            else
            {
                calib_min[i] = (calib_max[i] + calib_min[i])/2; 
            }
            
            calibration_finished = (calib_max[i] - calib_min[i]) <= 1;
        }        
    }
    //mpu_.setDMPEnabled(true);
    mpu_.resetDMP();   
}
}