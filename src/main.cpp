

#include <Arduino.h>
#include <Infrared.h>
#include <Motor.h>
#include <Ultrasonic.h>

Ultrasonic uSonic;
Infrared IR;
Motor motors(P0_4, P0_5, P0_27, P1_2);

//Timeout function exists to prevent robot from getting stuck in a move it can't complete
Timeout timeout_main;
volatile bool timeout_main_occurred = false;
void onTimeoutMain() { timeout_main_occurred = true; }
void beginTimeoutMain(float time_main) {
    timeout_main.detach();
    timeout_main.attach(&onTimeoutMain, time_main);
}


/**
 * @brief uses a combination of IRsensors and Motors to align the robot paralell to a wall
 *
 * @param mode an integer values that defines whether the wall that is being aligned
 * too is on the left or the right through use of different sensors
 * 1: left wall alignment and left pair of sensors
 * 2: right wall alignment and right pair of sensors
 */
void wallAlign(int mode) {
    float const LEEWAY = 0.3;
    // as the IRsensors will have natural variance between eachother this constant represents
    // how much they can differ and still be accepted as "aligned"
    timeout_main_occurred = false;
    beginTimeoutMain(5); // starts a 5 second timer where if wall alignments hasn't succeeded in 5 seconds stops trying to wall align
    if (mode == 1) {
        if ((IR.IRSensor(1) < 15) && IR.IRSensor(3) < 15) {
            while (IR.IRSensor(1) != IR.IRSensor(3)) { // code for left hand wall alignment
                float frontLeft = IR.IRSensor(1);
                float backLeft = IR.IRSensor(3);
                if (frontLeft == backLeft) { // if these are equal toeachother then wall has been aligned too so exit the loop
                    break;
                }
                if (frontLeft > backLeft + LEEWAY) { // if frontleft is sensor is further away from the wall than backleft,
                                                     // robot slowly turns anticlockwise/left to allign itself with the wall
                    motors.turnAngle(-1);
                } else if (backLeft > frontLeft + LEEWAY) { // opposite of above, if backleft is further away then robot turns
                                                            // right/clockwise to allign itself with the wall
                    motors.turnAngle(1);
                }
                if (timeout_main_occurred) {
                    // robot has gotten stuck and so the movement has been forcefully terminated
                    timeout_main_occurred = false; // resets timeout state to false
                    break;
                }
            }
        }
    } else {
        if ((IR.IRSensor(0) < 15) && IR.IRSensor(2) < 15) {
            while (IR.IRSensor(0) != IR.IRSensor(2)) { // code for right hand wall alignment, the same but uses different sensors and flips way
                                                       // it turns as otherwise sensors would be turning away from the wall
                float frontRight = IR.IRSensor(0);
                float backRight = IR.IRSensor(2);
                if (frontRight == backRight) {
                    break;
                }
                if (frontRight > backRight + LEEWAY) {
                    motors.turnAngle(1);
                } else if (backRight > frontRight + LEEWAY) {
                    motors.turnAngle(-1);
                }
                if (timeout_main_occurred) {
                    // robot has gotten stuck and so the movement has been forcefully terminated
                    timeout_main_occurred = false; // resets timeout state to false
                    break;
                }
            }
        }
    }
    motors.motorStop(); // stops both motors after sensors are equal eachother
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    motors.bothMotorSetup();
    // initialises and writes to the R,G,B LED pins on the ardunino board
    digitalWrite(LEDG, LOW);  // have the green LED be the only one that's low/on
    digitalWrite(LEDB, HIGH); // have the blue and red LED be off/high
    digitalWrite(LEDR, HIGH);
}

void explore() {
    const float IR_TOO_CLOSE = 20;
    const float USONIC_TO_CLOSE = 6;
    const float US_INVALID = -1;
    bool frontClear, frontLeftClear, backLeftClear, frontRightClear, backRightClear;

    while (true) {
        float distanceFront = uSonic.uSonicSensor();
        float frontRight = IR.IRSensor(0);
        float frontLeft = IR.IRSensor(1);
        float backRight = IR.IRSensor(2);
        float backLeft = IR.IRSensor(3);
        frontClear = !(distanceFront < USONIC_TO_CLOSE || distanceFront == US_INVALID);
        frontLeftClear = !(frontLeft < IR_TOO_CLOSE);
        backLeftClear = !(backLeft < IR_TOO_CLOSE);
        frontRightClear = !(frontRight < IR_TOO_CLOSE);
        backRightClear = !(backRight < IR_TOO_CLOSE);

        if ((motors.yCoordMode == 1) && (distanceFront > 10)) {
            // checks if there is space ahead and that robot is facing goal if both are true then it goes forward
            motors.move(10);
            return;
        }
        if (!frontClear) {
            // nothing in front after so move forward
            motors.move(-7);
        } else {
            if (frontLeftClear && backLeftClear) {
                // nothing on the left so turn left
                if (motors.currentXCoord > 10) { // checks if motor is on the far left hand side of the maze if it is then prevents left turn
                    motors.turnLeft();
                    wallAlign(0); // wall aligns with left sensors
                    motors.move(25);
                } else {
                    motors.turnRight();
                    wallAlign(1); // wall aligns with right sensors
                }
                if (uSonic.uSonicSensor() > 15) {
                    // nothing in front after turning so move forward
                    motors.move(10);
                }
            } else if ((frontLeftClear && !backLeftClear) || (!frontLeftClear && backLeftClear)) {
                // Robot is about to clear a wall as one sensor has past it, move forward
                // larger movement to account for slight wheel turning failure and to ensure all the robot is clear
                motors.move(20);

            } else {
                // Blocked on left
                // Check if the front is clear enough to move forward
                if (distanceFront > 15) {
                    // checks again if some movement can be made
                    motors.move(5);
                } else {
                    if (frontRightClear && backRightClear) {
                        // as left turn hasn't worked try turning right
                        motors.turnRight();
                        wallAlign(1);
                        if (uSonic.uSonicSensor() > 10) {
                            // after turning right if there is space ahead go forward
                            motors.move(7);
                        }
                    } else if (frontRightClear && !backRightClear) {
                        // Robot is about to clear a wall as one sensor has past it, move forward
                        // larger movement to account for slight wheel turning failure and to ensure all the robot is clear
                        motors.move(20);
                    } else {
                        // robot's only movement option is go backwards on it's self
                        motors.turnAround();
                        wallAlign(1);
                    }
                }
            }
        }
        thread_sleep_for(100);
        //small delay mainly added for decision making tree checking and to allow sensor readings to be more stable
    }
}
void loop() {
    explore();
}
