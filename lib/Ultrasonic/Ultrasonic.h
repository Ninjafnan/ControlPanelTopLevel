/**
 * Ultrasonic sensor code adapted from the arduino ide example code for the Ping ultrasonic sensor  (File -> Examples -> Sensors -> Ping), with a bit of help from my collegue Will Beverley
 * created 3 Nov 2008 by David A. Mellis
 * modified 30 Aug 2011 by Tom Igoe
 * This example code is in the public domain.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>
#include <mbed.h>
using namespace mbed;
#pragma once

class Ultrasonic {
public:
    /**
     *@brief Construct a new Ultrasonic object
     */
    long uSonicSensor();

private:
    /**
     *@brief Establish variables for duration of the sensor, and the subesquent returned distance in centimeters:
     */
    long duration, distance;
    /**
     *@param USONIC_PIN due to my design only containing 1 ultrasonic the below ultrasonic connecting pin is a constant
     */
    const int USONIC_PIN = 6;
    /**
     *@param CONVERT_TO_CM_FACTOR the constant required to convert the "duration" into a distance in cm
     */
    const int CONVERT_TO_CM_FACTOR = 58;
};

#endif