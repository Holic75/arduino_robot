
#ifndef ARDUINO_DC_MOTOR_H
#define ARDUINO_DC_MOTOR_H

#if defined(ARDUINO)
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif


namespace arduino_robot
{
class DCMotor
{ 
	uint8_t  control_pin_;
	uint8_t  pwm_pin_;
	uint8_t  power0_;
	bool inverse_control_;

public:
	DCMotor(uint8_t control_pin, uint8_t pwm_pin, bool inverse_control = false, uint8_t power0 = 0);
	void run(uint8_t pwm, bool direction);
	void run(int16_t pwr);
	void runPrct(int8_t pwr_prct);
	void stop();
	bool isInversed() const { return inverse_control_; };
};

namespace aux
{
class EncoderIsrPool
{

public:
	class PulseCounter
	{	
	friend class EncoderIsrPool;
		void (*isr_) ();
		uint8_t pin;
	public:
		long pulse_count;
		bool increment_pulse;
		PulseCounter(void (*isr)())
			:isr_(isr), pin(0), pulse_count(0){};		
	};
private:
    static const uint8_t MAX_NUM_INTERRUPTS_ = 5;
	static PulseCounter  pulse_counters_[MAX_NUM_INTERRUPTS_];
	template<uint8_t isr_id> static void isr()
	{
		if (pulse_counters_[isr_id].increment_pulse)
		{
			pulse_counters_[isr_id].pulse_count++;
		}
		else
		{
			pulse_counters_[isr_id].pulse_count--;
		}
	};
	
public:
	static PulseCounter* attachInterrupt(uint8_t interrupt_pin);
	static void free(PulseCounter* pulse_counter_);
	EncoderIsrPool() = delete;
};
	
}

class DCMotorEncoder: public DCMotor
{ 
	aux::EncoderIsrPool::PulseCounter* pulse_counter_ = nullptr;

public:
	DCMotorEncoder(uint8_t control_pin, uint8_t pwm_pin, uint8_t encoder_pin, bool inverse_control = false, uint8_t power0 = 0);
	void run(uint8_t pwm, bool direction);
	void run(int16_t pwr);
	void runPrct(int8_t pwr_prct);

	bool isOk();
	void reset();
	long getPulseCount() const;
    
    DCMotorEncoder(const DCMotorEncoder&) = delete;
	
	~DCMotorEncoder();
};
}





#endif

