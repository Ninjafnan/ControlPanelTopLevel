/**
 * library for the GP2Y0E02B Infrared sensor, the mentioned data sheet can be here,
 * https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0e02_03_appl_e.pdf.
 */
#ifndef INFRARED_H
#define INFRARED_H
#include <Arduino.h>
#include <mbed.h>
using namespace mbed;
#pragma once

class Infrared {
public:
    /**
     * @brief Construct a new IRSensor with it's corresponding sensor port
     * 
     * @param SensorNo varies from 0-3, changes which IRSensor should be read
     *
     */
    float IRSensor(char SensorNo);

private:
    const char MUX_ADDR = 0xEE; //constant address in memory for the mux
    char cmd[2]; //array to hold readings
    uint16_t temp; //temp value for to store readed values before conversion to cm
    float sensedValue; //returned float value
    const char SENSOR_ADDR = 0x80; //constant address for the sensor
};

#endif