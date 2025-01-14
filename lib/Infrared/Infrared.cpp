
#include <Infrared.h>
I2C i2c(P0_31, P0_2);


float Infrared::IRSensor(char SensorNo) {

    const char MUX_CMD = 1 << SensorNo; // uses the inputted to change which multiplexer channel is being used and therefore which IR Sensor is being written too/read

    i2c.write(MUX_ADDR, &MUX_CMD, 1);
    cmd[0] = 0x5E;
    cmd[1] = 0x00;
    i2c.write(SENSOR_ADDR, cmd, 1);
    wait_us(500); // required delay
    i2c.read(SENSOR_ADDR, cmd, 2);

    temp = ((cmd[0] << 4) + cmd[1]);                 // formula from sensor datasheet to take raw data and makes them useful
    sensedValue = static_cast<float>(temp) / 16 / 4; // converts value into the actual distance desired in CM

    Serial.print(SensorNo, HEX);
    Serial.print(" ");
    Serial.print(sensedValue);
    Serial.print(" cm \n ");

    return sensedValue;
}
