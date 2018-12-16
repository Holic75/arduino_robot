

#ifndef HCSR04_H
#define HCSR04_H

#include "Arduino.h"

namespace arduino_robot
{
class UltraSonicDistanceSensor 
{
 public:
    /**
     * @param triggerPin  Digital pin that is used for controlling sensor (output).
     * @param echoPin  Digital pin that is used to get information from sensor (input).
     */
    UltraSonicDistanceSensor(int triggerPin, int echoPin);

    /**
     * Measures distance by sending ultrasonic waves and measuring time it takes them
     * to return.
     * @returns Distance in centimeters, or negative value if distance is greater than 400cm.
     */
    float measureDistanceCm();
 private:
    int triggerPin, echoPin;
    static const float SOUND_SPEED_MS_ = 343.21f; //at 20 degreeC
    static const float DELAY_MCS_DISTANCE_CM_CONVERSION_COEFF_ = 1e-4f*SOUND_SPEED_MS_/2.0f;
    static const unsigned long MAX_DELAY_MCS_ = 400/DELAY_MCS_DISTANCE_CM_CONVERSION_COEFF_;
};
}
#endif // HCSR04_H
