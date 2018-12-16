#include "Arduino.h"
#include "HCSR04.h"


namespace arduino_robot
{
UltraSonicDistanceSensor::UltraSonicDistanceSensor(
        int triggerPin, int echoPin) 
{
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float UltraSonicDistanceSensor::measureDistanceCm() 
{
    // Make sure that trigger pin is LOW.
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
    unsigned long duration_mcs = pulseIn(echoPin, HIGH, MAX_DELAY_MCS_);
    
    if (duration_mcs == 0) 
    {
        return -1.0f ;
    } 
    else 
    {
        float distance_cm = duration_mcs * DELAY_MCS_DISTANCE_CM_CONVERSION_COEFF_;
        return distance_cm;
    }
}
}
