
#include "DCMotor.h"

using namespace arduino_robot;

aux::EncoderIsrPool::PulseCounter aux::EncoderIsrPool::pulse_counters_[aux::EncoderIsrPool::MAX_NUM_INTERRUPTS_] = 
                       {aux::EncoderIsrPool::PulseCounter(&aux::EncoderIsrPool::isr<0>),
                       aux::EncoderIsrPool::PulseCounter(&aux::EncoderIsrPool::isr<1>),
                       aux::EncoderIsrPool::PulseCounter(&aux::EncoderIsrPool::isr<2>),
                       aux::EncoderIsrPool::PulseCounter(&aux::EncoderIsrPool::isr<3>),
                       aux::EncoderIsrPool::PulseCounter(&aux::EncoderIsrPool::isr<4>)};


DCMotor::DCMotor(uint8_t control_pin, uint8_t pwm_pin, bool inverse_control, uint8_t power0)
	:control_pin_(control_pin), pwm_pin_(pwm_pin), inverse_control_(inverse_control), 
    power0_(power0) 
    
{
    pinMode(control_pin, OUTPUT);
	pinMode(pwm_pin, OUTPUT);
};


void DCMotor::run(uint8_t pwm, bool direction)
{
	if (direction != inverse_control_)
	{
		digitalWrite(control_pin_, HIGH);
	}
	else
	{
		digitalWrite(control_pin_, LOW);
	}
	analogWrite(pwm_pin_, pwm);
}

void DCMotor::run(int16_t pwr)
{
	run(abs(pwr), pwr > 0);
}

void  DCMotor::runPrct(int8_t pwr_prct)
{
    if (pwr_prct == 0)
    {
        stop();
        return;
    }
    uint8_t d_power_prct = 255u - power0_;
	run(power0_ + abs(pwr_prct)*d_power_prct/100u, pwr_prct > 0);
}

void DCMotor::stop()
{
	analogWrite(pwm_pin_, 0);
}





aux::EncoderIsrPool::PulseCounter* aux::EncoderIsrPool::attachInterrupt(uint8_t interrupt_pin)
{
	for (uint8_t i = 0; i < MAX_NUM_INTERRUPTS_; i++)
	{
		if (pulse_counters_[i].pin == 0)
		{
			pinMode(interrupt_pin, INPUT);
			::attachInterrupt(digitalPinToInterrupt(interrupt_pin), pulse_counters_[i].isr_, RISING);
			pulse_counters_[i].pin = interrupt_pin;
			pulse_counters_[i].pulse_count = 0;
			return &pulse_counters_[i];
		}
	}

	return nullptr;
}


void aux::EncoderIsrPool::free(aux::EncoderIsrPool::PulseCounter* pulse_counter_)
{
	for (uint8_t i = 0; i < MAX_NUM_INTERRUPTS_; i++)
	{
		if (&pulse_counters_[i] == pulse_counter_)
		{
			detachInterrupt(pulse_counter_->pin);
			pulse_counter_->pin = 0;
			return;
		}
	}

	return;
}


DCMotorEncoder::DCMotorEncoder(uint8_t control_pin, uint8_t pwm_pin, uint8_t encoder_pin, bool inverse_control, uint8_t power0)
	:DCMotor(control_pin, pwm_pin, inverse_control, power0),
    pulse_counter_(aux::EncoderIsrPool::attachInterrupt(encoder_pin))
{
   
}


void DCMotorEncoder::run(uint8_t pwm, bool direction)
{
	DCMotor::run(pwm, direction);
	pulse_counter_->increment_pulse = direction != isInversed();
}


void DCMotorEncoder::run(int16_t pwr)
{
	run(abs(pwr), pwr > 0);
}


void DCMotorEncoder::runPrct(int8_t pwr_prct)
{
    DCMotor::runPrct(pwr_prct);
	pulse_counter_->increment_pulse = (pwr_prct > 0) != isInversed();
}


bool DCMotorEncoder::isOk()
{
	return pulse_counter_ != nullptr;
}


void DCMotorEncoder::reset()
{
	pulse_counter_->pulse_count = 0;
}


long  DCMotorEncoder::getPulseCount() const
{
	return pulse_counter_->pulse_count;
}
	
DCMotorEncoder::~DCMotorEncoder()
{
	aux::EncoderIsrPool::free(pulse_counter_);
}