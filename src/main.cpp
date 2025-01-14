



#include <Arduino.h>
#include <Infrared.h>
#include <Motor.h>
#include <Ultrasonic.h>

Ultrasonic uSonic;
Infrared IR;
Motor motors(P0_4, P0_5, P0_27, P1_2);

// Timeout timeout_main;
// volatile bool timeout_main_occurred = false;
// void onTimeoutMain() { timeout_main_occurred = true; }
// void beginTimeoutMain(float time_main) {
//     timeout_main.detach();
//     timeout_main.attach(&onTimeoutMain, time_main);
// }

/**
 * @brief uses a combination of IRsensors and Motors to align the robot paralell to a wall
 *
 * @param mode an integer values that defines whether the wall that is being aligned
 * too is on the left or the right through use of different sensors
 * 1: left wall alignment and left pair of sensors
 * 2: right wall alignment and right pair of sensors
 */
void wallAlign(int mode) {
    float const LEEWAY = 0.3; // as the IRsensors will have natural variance between eachother this constant represents how much they can differ and still be accepted as "aligned"
    //beginTimeoutMain(5);
    if (mode == 1) {
        while (IR.IRSensor(1) != IR.IRSensor(3)) { // code for left hand wall alignment
            float frontLeft = IR.IRSensor(1);
            float backLeft = IR.IRSensor(3);
            if (frontLeft == backLeft) { // if these are equal toeachother then wall has been aligned too so exit the loop
                break;
            }
            if (frontLeft > backLeft + LEEWAY) { // if frontleft is sensor is further away from the wall than backleft, robot slowly turns anticlockwise/left to allign itself with the wall
                motors.turnAngle(-1);
            } else if (backLeft > frontLeft + LEEWAY) { // opposite of above, if backleft is further away then robot turns right/clockwise to allign itself with the wall
                motors.turnAngle(1);
            }
            // if (timeout_main_occurred) {
            //     // robot has gotten stuck and so the movement has been forcefully terminated
            //     break;
            // }
        }
    } else {
        while (IR.IRSensor(0) != IR.IRSensor(2)) { // code for right hand wall alignment, the same but uses different sensors and flips way it turns as otherwise sensors would be turning away from the wall
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
            // if (timeout_main_occurred) {
            //     // robot has gotten stuck and so the movement has been forcefully terminated
            //     break;
            // }
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


    float distanceAhead = uSonic.uSonicSensor();
    float frontRight = IR.IRSensor(0);
    float frontLeft = IR.IRSensor(1);
    float backRight = IR.IRSensor(2);
    float backLeft = IR.IRSensor(3);

    if (distanceAhead < 6 || distanceAhead == -1 || distanceAhead > 700) {
        motors.move(-2);
    } else {
        if ((frontLeft > 10) && (backLeft > 10)) {
            // no wall on the left therefore must have moved past a wall turn left to continue left hand following
            motors.turnLeft();
            motors.move(10);

        } else if ((frontLeft > 10) && ((backLeft < 10))) {
            // robot is not quite yet done avoiding the wall to be able to turn left so move forward a bit and then turn left and move
            motors.move(5);
            motors.turnLeft();
            motors.move(10);
        } else {
            // full blocked on the left
            if (distanceAhead < 20) {
                // check if you can move forward in any capacity
                motors.move(5);
            } else {
                if ((frontRight > 10) && (backRight > 10)) {
                    // nothing on the right and as we have already checked left hand we want to turn right
                    motors.turnRight();
                    motors.move(10);
                } else if ((frontRight > 10) && (!(backRight > 10))) {
                    motors.move(5);
                    // motors.turnRight(); don't turn right immediately incase moving forwards causes left to be unblocked
                } else {
                    motors.turnAround();
                    wallAlign(1);
                }
            }
        }
    }
}

void exploreNew() {
    const float CLOSE_TO_SIDE_WALL_DISTANCE = 15;
    const float FAR_TOO_CLOSE_DISTANCE = 6;
    const float US_FAIL = -1;
    bool frontClear, leftFrontClear, leftBackClear, rightFrontClear, rightBackClear;
    // const int TargetforwardToEnd = 1950;

    while (true) {
        float distanceFront = uSonic.uSonicSensor();
        float frontRight = IR.IRSensor(0);
        float frontLeft = IR.IRSensor(1);
        float backRight = IR.IRSensor(2);
        float backLeft = IR.IRSensor(3);
        frontClear = !(distanceFront < FAR_TOO_CLOSE_DISTANCE || distanceFront == US_FAIL);
        leftFrontClear = !(frontLeft < CLOSE_TO_SIDE_WALL_DISTANCE);
        leftBackClear = !(backLeft < CLOSE_TO_SIDE_WALL_DISTANCE);
        rightFrontClear = !(frontRight < CLOSE_TO_SIDE_WALL_DISTANCE);
        rightBackClear = !(backRight < CLOSE_TO_SIDE_WALL_DISTANCE);


        if ((motors.yCoordMode == 1) && (distanceFront > 10))  {
            motors.move(10);
            return;
        }

        if (!frontClear) {
            // Blocked on front -> move backwards
            motors.move(-7);
        } else {
            if (leftFrontClear && leftBackClear) {
                // Clear on left -> turn left
                if (motors.currentXCoord > 10) {
                    motors.turnLeft();
                } else {
                    motors.turnRight();
                }
                if (uSonic.uSonicSensor() > 15) {
                    // Clear in front -> move forward
                    motors.move(10);
                }
                // wallAlign(1);
            } else if ((leftFrontClear && !leftBackClear) || (!leftFrontClear && leftBackClear)) {
                // Close to clearing left wall -> move forward enought to fully clear -> turn left
                motors.move(16);
            } else {
                // Blocked on left
                // Check if the front is clear enough to move forward
                if (distanceFront > 15) {
                    // Clear in front -> move forward
                    motors.move(5);
                } else {
                    if (rightFrontClear && rightBackClear) {
                        // Clear on right -> turn right
                        motors.turnRight();
                        if (uSonic.uSonicSensor() > 10) {
                            // Clear in front -> move forward
                            motors.move(15);
                        }
                        wallAlign(1);
                    } else if (rightFrontClear && !rightBackClear) {
                        // Close to clearing right wall -> move forward
                        motors.move(18);
                    } else {
                        // Fully blocked on front, left and right -> turn 180
                        motors.turnAround();
                        wallAlign(1);
                    }
                }
            }
        }

        thread_sleep_for(100);
    }
}
void loop() {


    // put your main code here, to run repeatedly:
    // uSonic.uSonicSensor();
    // IR.IRSensor(0);
    // IR.IRSensor(1);
    // IR.IRSensor(2);
    // IR.IRSensor(3);
    // motors.moveMotors(0,0.5,0,0.5);

   
    //     if (FrontWall < 13 || FrontWall > 600){
    //   ReverseThisFar(10.0);
    // }
    // else{
    //   if(LeftWall > 35 && LeftWall < 900){ // Check if there is a wall on the left using ultrasonic. there isn't
    //     float FL = AverageIRSensor(FRONT_LEFT); // Check front left ir
    //     float BL = AverageIRSensor(BACK_LEFT); // Check back left ir
    //     if(FL > 30 && BL > 30){ // Check whether both sensors confirm there is no wall
    //       TurnLeft(); // If there is a wall close in front, do a small turn to avoid driving into it.
    //       GoThisFar(340.0); // Go forwards 10 cm and check again
    //     }
    //     else if((FL > 20 && BL < 20) || (FL <20 && BL > 20)){ // Is there is a wall in front of one sensor but not the other
    //       if(FrontWall > 10 && FrontWall < 900){ // Check if there is a wall close in front, in this case there isn't
    //         GoThisFar(50.0); // Go forwards a little bit
    //       }
    //       else if(FrontWall <= 13){ // In this case there is a wall close in front
    //         ReverseThisFar(10.0); // Reverse a little
    //       }
    //       else{
    //         TurnRight();
    //       }
    //     }
    //     else{ // There actually is a wall on the left, the ultrasonic just failed.
    //       SquareUp();
    //       if(FrontWall > 13 && FrontWall < 900){ // Check if there is a wall close in front. In this case there isn't
    //         GoThisFar(100.0f); // Go forwards a bit and check again
    //       }
    //       else if(FrontWall <= 14 || FrontWall > 900){
    //         TurnRight();
    //       }
    //     }
    //   }
    //   else if(LeftWall <= 25 || LeftWall >900){ // Check if there is a wall on the left using ultrasonic. there is
    //     if(FrontWall >= 16 && FrontWall < 900){ // Check if there is a wall close in front. there isn't
    //       GoThisFar(100.0f); // Go forwards and check again
    //     }
    //     else if(FrontWall < 16){ // Check if there is a wall close in front. there is
    //       if(RightWall > 20 && RightWall <900){ // Check if there is a wall on the Right using ultrasonic. there isn't
    //         float FR= AverageIRSensor(FRONT_RIGHT); // Check front Right ir
    //         float BR= AverageIRSensor(BACK_RIGHT); // Check back Right ir
    //         if(FR> 20 && BR> 20){ // Check whether both sensors confirm there is no wall
    //           TurnRight();
    //         }
    //         else if(FR> 20 && BR< 20){ // If there is a wall in front of back sensor but not front. I don't think this can even happen but just in case.
    //           if(FrontWall > 10 && FrontWall < 900){ // Check if there is a wall close in front
    //             GoThisFar(50.0); // Go forwards a little bit
    //           }
    //         }
    //         else if (FR<20 && BR> 20){ // If there is a wall in front of front sensor but not back
    //           ReverseThisFar(50.0);
    //         }
    //         else{
    //           if(FrontWall >= 16 && FrontWall < 900){ // Check if there is a wall close in front
    //             GoThisFar(100.0f); // Go forwards and check again
    //             SquareUp(); // Square up with the closest wall
    //           }
    //         }
    //       }
    //       else{ // There is a wall on the left, the front and the right
    //         TurnRight(); // Turn around 180 degrees
    //         TurnRight();
    //       }
    //     }
    //   }
    // }

    // float distanceAhead = uSonic.uSonicSensor();
    // float frontRight = IR.IRSensor(0);
    // float frontLeft = IR.IRSensor(1);
    // float backRight = IR.IRSensor(2);
    // float backLeft = IR.IRSensor(3);
    // if ((distanceAhead > 10 || distanceAhead < 700) && ((frontLeft < 10) && (backLeft < 10))) { //if there is wall not in front and IR sensors on left detect a wall
    //     motors.move(5);
    //     if (distanceAhead < 6 || distanceAhead == -1 || distanceAhead > 700) {
    //         motors.move(-2); //if your too close to a wall move back a bit
    //         if ((frontLeft > 10) && (backLeft > 10)) {
    //             // no wall on the left therefore must have moved past a wall turn left to continue left hand following
    //             motors.turnLeft();
    //             motors.move(10);

    //             // Is there is a wall in front of one sensor but not the other
    //         } else if ((frontLeft > 10 && backLeft < 10) || (frontLeft < 10 && backLeft > 10)) {

    //             if (distanceAhead > 10 || distanceAhead < 700) { // Checks if there is a wall close in front, in this case there isn't
    //                 motors.move(5);
    //             } else if (distanceAhead <= 13) { // In this case there is a wall close in front
    //                 motors.move(-10);             // reverse a bit
    //             } else {
    //                 motors.turnRight();
    //             }
    //         } else if (frontRight > 10 && backRight > 10) {
    //             motors.turnRight();
    //         } else if (frontRight > 10 && backRight < 10) {
    //             motors.move(-5);
    //         } else {
    //             if (distanceAhead > 10 || distanceAhead < 700) {
    //                 motors.move(10);
    //             }
    //         }
    //         motors.turnAround();
    //     }
    // }

    
    //motors.move(1000);

    // explore();
    //motors.move(10);
    // Serial.println((String) "now finished" ) ;
    // delay(10000);
    // Serial.println((String) "now starting" ) ;
    //motors.moveMotors(0, 0.5 , 0, 0.5);
    //exploreNew();
    motors.turnLeft();

    // motors.turnLeft();
    // delay(10000);
    // motors.turnAngle(1);
    //  Serial.println("hihihihihi");
    // motors.rampSpeed(1 , 1);
    // wallAlign(true);
    // delay(10000);

    delay(100000);
}
