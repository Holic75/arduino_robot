#include "smart_servo.h"
#include <Arduino.h>

namespace arduino_robot
{
SmartServo::SmartServo(uint8_t pos_center, int8_t pos_min, int8_t pos_max, uint8_t servo_pin)
	:pos_center_(pos_center), pos_min_(pos_min), pos_max_(pos_max), servo_pin_(servo_pin),
	min_pulse_width_(MIN_PULSE_WIDTH), max_pulse_width_(MAX_PULSE_WIDTH)
{
	pinMode(servo_pin_, OUTPUT);
	//attach();
}


void SmartServo::setPulseWidth(int16_t min_pulse_width, int16_t max_pulse_width)
{
	max_pulse_width_ = max_pulse_width;
	min_pulse_width_ = min_pulse_width;	

	if (servo_.attached())
	{
		detach();
		attach();
	}
}


void SmartServo::set(int8_t pos)
{
	if (pos > pos_max_)
	{
		pos = pos_max_;
	}
	else if (pos < pos_min_)
	{
		pos = pos_min_;
	}

	servo_.write(pos_center_ + pos);
}


int8_t SmartServo::requestedPosition() const
{
	return servo_.read() - pos_center_;
}


void SmartServo::detach()
{
	servo_.detach();
}


void SmartServo::attach()
{
	servo_.attach(servo_pin_, min_pulse_width_, max_pulse_width_);
}




/////////////////////////////////////////////////////////////
FeedbackServo::FeedbackServo(uint8_t pos_center, int8_t pos_min, int8_t pos_max, uint8_t servo_pin, uint8_t pot_pin, uint16_t pot_min, uint16_t pot_max)
	:SmartServo(pos_center, pos_min, pos_max, servo_pin), pot_pin_(pot_pin), pot_min_(pot_min), pot_max_(pot_max)
{
	pinMode(pot_pin_, INPUT);
}


int16_t FeedbackServo::actualPosition() const
{
	int16_t pot_value = analogRead(pot_pin_);
	return  (pot_value - pot_min_) * static_cast<int32_t>(pos_max_ - pos_min_) / (pot_max_ - pot_min_) + pos_min_;
}


void FeedbackServo::calibrate(long delay_msec)
{
	attach();
	set(pos_min_);
	delay(delay_msec);
	pot_min_ = analogRead(pot_pin_);
	set(pos_max_);
	delay(delay_msec);
	pot_max_ = analogRead(pot_pin_);
}
}








