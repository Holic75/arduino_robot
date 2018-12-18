#include "smart_servo.h"



SmartServo::SmartServo(uint8_t pos_center, int8_t pos_min, int8_t pos_max, uint8_t servo_pin)
    :pos_center_(pos_center), pos_min_(pos_min), pos_max_(pos_max), servo_pin_(servo_pin)
{
    pinMode(servo_pin_, OUTPUT);
    attach();
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


int8_t SmartServo::requestedPosition()
{
    return servo_.read() - pos_center_;
}


void SmartServo::detach()
{
    servo_.detach();
}


void SmartServo::attach()
{
    servo_.attach(servo_pin_);
}




/////////////////////////////////////////////////////////////
FeedbackServo::FeedbackServo(uint8_t pos_center, int8_t pos_min, int8_t pos_max, uint8_t servo_pin, uint8_t pot_pin, uint16_t pot_min, uint16_t pot_max)
    :SmartServo(pos_center, pos_min, pos_max, servo_pin), pot_pin_(pot_pin), pot_min_(pot_min), pot_max_(pot_max) 
{
    pinMode(pot_pin_, INPUT);
}


int8_t FeedbackServo::actualPosition() const
{
    uint16_t pot_value = analogRead(pot_pin_);
    return  (pot_value - pot_min_) * (pos_max_ - pos_min_) / (pot_max_ - pot_min_) + pos_min_;
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








