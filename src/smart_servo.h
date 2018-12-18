#ifndef AR_SMART_SERVO_H
#define AR_SMART_SERVO_H


#include <Servo.h>


namespace arduino_robot
{
    
class SmartServo
{
protected:
    Servo servo_;
    uint8_t pos_center_;
    int8_t pos_min_;
    int8_t pos_max_;
    uint8_t servo_pin_;
public:
    SmartServo(uint8_t pos_center, int8_t pos_min, int8_t pos_max, uint8_t servo_pin);
    void set(int8_t pos);
    int8_t requestedPosition() const;
    void detach();
    void attach();
};


class FeedbackServo : public SmartServo
{
private:
    uint8_t pot_pin_;
    uint16_t pot_min_;
    uint16_t pot_max_;
public:
    FeedbackServo(uint8_t pos_center, int8_t pos_min, int8_t pos_max, uint8_t servo_pin, uint8_t pot_pin, uint16_t pot_min = 0, uint16_t pot_max = 1023);
    int8_t actualPosition() const;
    void calibrate(long delay_msec = 1000);
};

}

#endif