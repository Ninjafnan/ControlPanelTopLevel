
#include "ultrasonic.h"

long Ultrasonic::uSonicSensor() {
    /** Establish variables for duration of the sensor, and the distance result in centimeters:
     *The sensor is triggered by a HIGH pulse of 2 or more microseconds.
     *a short LOW pulse is given beforehand to ensure a clean HIGH pulse:
     */
    pinMode(USONIC_PIN, OUTPUT);
    digitalWrite(USONIC_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(USONIC_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(USONIC_PIN, LOW);
    /** 
     * The same pin is used to read the signal from the sensor: a HIGH pulse
     * Whose duration is the time (in microseconds) from the sending of the ping
     * To the reception of its echo off of an object.
     */
    pinMode(USONIC_PIN, INPUT);
    duration = pulseIn(USONIC_PIN, HIGH);

    // Convert the time into a distance, which in cm requires a division of 58
    distance = duration / CONVERT_TO_CM_FACTOR;
    Serial.println((String) "Usonic: " + distance);
    delay(100);
    return distance;
}