

#ifndef IMU6050_WRAPPER_H
#define IMU6050_WRAPPER_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


namespace arduino_robot
{
class IMU6050
{
	private:
		// MPU control/status vars
		  // set true if DMP init was successful
		bool dmpReady_ = false;
		uint8_t mpuIntStatus_;   // holds actual interrupt status byte from MPU
		uint16_t packetSize_;    // expected DMP packet size (default is 42 bytes)
		uint16_t fifoCount_;     // count of all bytes currently in FIFO
		uint8_t fifoBuffer[64]; // FIFO storage buffer

		MPU6050 mpu_;
		int16_t accel_low_filter_ = 0;

		// orientation/motion vars
		Quaternion q_;           // [w, x, y, z]         quaternion container
		float euler_[3];         // [psi, theta, phi]    Euler angle container
		float ypr_[3];           // [yaw, pitch, roll]   yaw/pitch/roll container
		VectorInt16 aa_;         // [x, y, z]            accel sensor measurements
        VectorInt16 w_;          // [wx, wy, wz]         gyro sensor measurements
		VectorInt16 aaReal_;     // [x, y, z]            gravity-free accel sensor measurements
		VectorInt16 aaWorld_;    // [x, y, z]            world-frame accel sensor measurements
		VectorFloat gravity_;    // [x, y, z]            gravity vector

        
	public:
		IMU6050(int i2c_address) :mpu_(i2c_address), dmpReady_(false) {};
		bool initialize(uint8_t gyro_sensetivity, uint8_t accel_sensetivity, VectorInt16 gyro_offset, VectorInt16 accel_offset, int16_t g_reference_value = 0);
		void set_accel_low_filter(int16_t accel_low_filter) { accel_low_filter_ = accel_low_filter; };
		void update();
        void calibrate();
        
        void getMotion(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) const;
        void clearBuffer() {mpu_.resetFIFO();};

        const VectorInt16& getAccelSensor() {return aa_;};
        const VectorInt16& getAccelLinear() {return aaReal_;};
        const VectorInt16& getGyroSensor() {return w_;};
        const VectorInt16& getAccelLinearWorld() {return aaWorld_;};
        
        const VectorFloat& getGravityVector() {return gravity_;};
		const Quaternion&  getQuaternion() { return q_; };
        
		float getYaw() const { return ypr_[0]; };
		float getPitch() const { return ypr_[1]; };
		float getRoll() const { return ypr_[2]; };
 
		float getEulerPsi() const { return euler_[0]; };
		float getEulerTheta() const { return euler_[1]; };
		float getEulerPhi() const { return euler_[2]; };
	
};
}

#endif