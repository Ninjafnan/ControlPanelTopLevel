#include <Arduino.h>
#include <Motor.h>

InterruptIn EncA(P1_11);
InterruptIn EncB(P1_12);

// Timeout timeout;
// volatile bool timeout_occurred = false;
// void onTimeout() { timeout_occurred = true; }
// void beginTimeout(float time) {
//     timeout.detach();
//     timeout.attach(&onTimeout, time);
// }

Motor::Motor(PinName MotorADir, PinName MotorBDir, PinName MotorAPWM, PinName MotorBPWM)
    : MotorADir(MotorADir), MotorBDir(MotorBDir), MotorAPWM(MotorAPWM), MotorBPWM(MotorBPWM) {}

void Motor::moveMotors(int directionA, float speedA, int directionB, float speedB) {
    MotorADir = directionA;
    MotorAPWM.write(speedA);
    MotorBDir = directionB;
    MotorBPWM.write(speedB);
}

void Motor::bothMotorSetup() {
    MotorAPWM.period_ms(1);
    MotorBPWM.period_ms(1);
}

void Motor::motorStop() {
    MotorAPWM.write(0);
    MotorBPWM.write(0);
}

void Motor::revolutionsA() {
    if (EncCountA % 6 == 0) {
        if (EncCountA % 110 == 0) {
            ShaftRevA++;
        }
    }
}
void Motor::revolutionsB() {
    if (EncCountB % 6 == 0) {
        if (EncCountB % 110 == 0) {
            ShaftRevB++;
        }
    }
}
void Motor::countPulseA() {
    if (MotorADir == 0) {
        EncCountA++;
    } else {
        EncCountA--;
    }
    revolutionsA();
}
void Motor::countPulseB() {
    if (MotorBDir == 0) {
        EncCountB++;
    } else {
        EncCountB--;
    }
    revolutionsB();
}

void Motor::rampSpeed(float targetSpeed, int type) {
    float rampFactor = 0.0005;
    float currentSpeed = 0.00000;
    int wantedDirA;
    int wantedDirB;

    if (type == 1) {
        wantedDirA = 0;
        wantedDirB = 0;
    } else if (type == 2) {
        wantedDirA = 1;
        wantedDirB = 1;
    } else if (type == 3) {
        wantedDirA = 1;
        wantedDirB = 0;
    } else if (type == 4) {
        wantedDirA = 0;
        wantedDirB = 1;
    } else {
        wantedDirA = 0;
        wantedDirB = 0;
    }

    EncCountA = 0; // resets encoder values to increase realiablity by removing any potential builtup errors
    EncCountB = 0;
    //beginTimeout(10);
    while (currentSpeed < targetSpeed) {
        currentSpeed = currentSpeed + rampFactor;
        moveMotors(wantedDirA, currentSpeed, wantedDirB, currentSpeed);
        Serial.println((String) "Current speed: " + currentSpeed + " Loop: " + timer);
        EncA.rise(callback(this, &Motor::countPulseA));
        EncB.rise(callback(this, &Motor::countPulseB));
        //Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " Timer: " + timer);
        timer++;

        if (abs(EncCountA) < abs(EncCountB)) {
            moveMotors(wantedDirA, currentSpeed, wantedDirB, 0);
        }
        if (abs(EncCountB) < abs(EncCountA)) {
            moveMotors(wantedDirA, 0, wantedDirB, currentSpeed);
        }
        if (abs(EncCountB) == abs(EncCountA)) {
            moveMotors(wantedDirA, currentSpeed, wantedDirB, currentSpeed);
        }
        // if (timeout_occurred) {
        //     // robot has gotten stuck and so the movement has been forcefully terminated
        //     break;
        // }
    }
    motorStop();
}
void Motor::move(int mili) {
    int wantedDirA = 0;
    int wantedDirB = 0;
    tempMili = mili;
    
    if (mili < 0) {
        // if mili is less than 0 it should be going backwards, otherwise it's going forwards
        wantedDirA = 1;
        wantedDirB = 1;
    } else if (0 < mili) {
        wantedDirA = 0;
        wantedDirB = 0;
    } else {
        return; // if 0 or somehow anything else is inputted that isn't allowed do nothing and exit the function
    }
    // beginTimeout(10);
    moveMotors(wantedDirA, 0.5, wantedDirB, 0.5);
    int TargetEncVal = mili * ENC_TICK_PER_MM * SCALE_FACTOR;
    Serial.println((String) "Target Encoder Value: " + TargetEncVal);
    EncCountA = 0; // resets encoder values to increase realiablity by removing any potential builtup errors
    EncCountB = 0;

    while ((abs(EncCountA) < abs(TargetEncVal)) || (abs(EncCountB) < abs(TargetEncVal))) {
        EncA.rise(callback(this, &Motor::countPulseA));
        EncB.rise(callback(this, &Motor::countPulseB));
        // Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " Timer: " + timer);
        timer++;
        if (abs(EncCountA) < abs(EncCountB)) {
            moveMotors(wantedDirA, 0.5, wantedDirB, 0);
        }
        if (abs(EncCountB) < abs(EncCountA)) {
            moveMotors(wantedDirA, 0, wantedDirB, 0.5);
        }
        if (abs(EncCountB) == abs(EncCountA)) {
            moveMotors(wantedDirA, 0.5, wantedDirB, 0.5);
        }
        // if (timeout_occurred) {
        //     // robot has gotten stuck and so the movement has been forcefully terminated
        //     Serial.println("timeout success");
        //     break;
        // }
    }
    motorStop();
    updateCoords();
    tempAngle = 0;
    action = MOVE;
    storeMovements();
    if (Exploring == false) {
        Serial.println("delay from explore false starts now");
        delay(1000);
    }
}

void Motor::checkOrientation() {
    switch (currentAngle) {
    case (0):
        yCoordMode = 1;
        xCoordMode = 0;
        break;
    case (90):
        yCoordMode = 0;
        xCoordMode = 1;
        break;
    case (180):
        yCoordMode = -1;
        xCoordMode = 0;
        break;
    case (-90):
        yCoordMode = 0;
        xCoordMode = -1;
        break;
    }
    Serial.println((String) " xCoordMode: " + xCoordMode + " yCoordMode: " + yCoordMode);
}
void Motor::updateCoords() {
    currentXCoord = currentXCoord + (tempMili * xCoordMode);
    currentYCoord = currentYCoord + (tempMili * yCoordMode);
    Serial.println((String) "xCoordMode: " + xCoordMode + " yCoordMode: " + yCoordMode + " xCoord: " + currentXCoord + " yCoord: " + currentYCoord);
    if (currentYCoord > 1950) {   // robot would have reached the end
        digitalWrite(LEDG, HIGH); // turn off green LED
        digitalWrite(LEDB, LOW);  // turn on Blue LED to indicate it's time to go back through the maze using array of stored moves
        Exploring = false;
        playbackMovements();
    }
}

void Motor::storeMovements() {
    aMove movement;
    movement.action = action;

    if (Exploring == true) {
        if (tempMili == 0) {
            movement.value = tempAngle;
        } else if (tempAngle == 0) {
            movement.value = tempMili;
        }
        history[historyCounter] = movement;
        historyCounter++;
        Serial.println((String) "movement action: " + movement.action + " movement value: " + movement.value + " historyCounter: " + historyCounter);
    }
}

void Motor::optimiseMovements() {
    int tempVal;
    aMove optimisedMovement;
    int j = 0;
    aMove prevMove;
    int prevVal;

    for (int i = historyCounter - 1; i >= 0; i--) {
        aMove movement;
        tempVal = 0;

        movement = history[i];
        while (movement.action == MOVE) {

            // movement = history[i];
            movement = history[i];

            tempVal = tempVal + optimisedMovement.value;
            Serial.println((String) " i val before: " + i);

            delay(1000);
            prevMove = history[i];
            prevVal = prevMove.value;
            Serial.println((String) " prevVal: " + prevVal + " i: " + i);
            i--;
            Serial.println((String) " i val after: " + i + " temp value momento: " + tempVal);
        }
        optimisedMovement.value = prevVal;
        optimisedMovement.action = MOVE;
        Serial.println((String) "optmovement action: " + optimisedMovement.action + " optmovement value: " + movement.value + " i: " + i);

        backToStart[j] = optimisedMovement;
        j++;
    }
}
void Motor::playbackMovements() {
    // Exploring = false;
    turnAround();
    for (int i = historyCounter - 1; i >= 0; i--) {
        aMove movement = history[i];
        Serial.println((String) "movement action: " + movement.action + " movement value: " + movement.value + " historyCounter: " + i);
        // thread_sleep_for(1000);
        if (movement.action == MOVE) {
            move(movement.value);
        } else if (movement.action == TURN) {
            turnAngle(movement.value);
        }
    }
}

void Motor::turnAngle(int angle) {
    int wantedDirA = 1;
    int wantedDirB = 0;
    float defaultSpeed = 0.5;
    // removes redundancies in any input angle as there no point in moving more than 360 degrees asit only introduces more encoder errors
    angle = angle % 360;
    Serial.println(angle);

    // makes the angle within the range of -180 < angle =< 180 for optimal turning direction
    if (180 < angle) {
        angle = angle - 360;
    }

    // effectively checking if it's doing very minute turning where a lower speed is better to prevent overshoot
    if ((-2 < angle) && (2 > angle)) {
        defaultSpeed = 0.2;
    }
    if (angle < 0) {
        // if angle is less than 0 it should be turning in the left direction, left meaning left of the ultrasonic if it's facing away from you as the quadrant is defined in this way...
        wantedDirA = 1;
        wantedDirB = 0;
    } else if (0 < angle) { //...otherwise it would be turning right and so these are used
        wantedDirA = 0;
        wantedDirB = 1;
    }

    EncA.rise(callback(this, &Motor::countPulseA));
    EncB.rise(callback(this, &Motor::countPulseB));
    //beginTimeout(10);
    // Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " Timer: " + timer);
    timer++;

    moveMotors(wantedDirA, defaultSpeed, wantedDirB, defaultSpeed); // starts motors moving as desired
    EncCountA = 0;                                                  // resets encoder values to increase realiablity by removing any potential builtup errors
    EncCountB = 0;
    int distanceTravelledInTurn = angle * ONE_DEGREE_IN_MM * SCALE_FACTOR;                  // calculate the target distance travel to travel
    int TargetEncVal = distanceTravelledInTurn * ENC_TICK_PER_MM * SCALE_FACTOR;            // find equivalent encoder value for that distance
    while ((abs(EncCountA) < abs(TargetEncVal)) || (abs(-EncCountB) < abs(TargetEncVal))) { // absolute values used so it doesn't matter which way the motors are turning
        EncA.rise(callback(this, &Motor::countPulseA));
        EncB.rise(callback(this, &Motor::countPulseB));
        // Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " target Enc:" + TargetEncVal + " Timer: " + timer);
        timer++;
        if (abs(EncCountA) < abs(-EncCountB)) { // syncs up encoders by momentarilly turning 1 of them off if the other goes ahead to ensure it spins on the spot
            moveMotors(wantedDirA, defaultSpeed, wantedDirB, 0);
        }
        if (abs(-EncCountB) < abs(EncCountA)) {
            moveMotors(wantedDirA, 0, wantedDirB, defaultSpeed);
        }
        if (abs(-EncCountB) == abs(EncCountA)) { // if encoders are synced they continue as desired
            moveMotors(wantedDirA, defaultSpeed, wantedDirB, defaultSpeed);
        }
        // if (timeout_occurred) {
        //     // robot has gotten stuck and so the movement has been forcefully terminated
        //     Serial.println("timeout success");
        //     break;
        // }
    }
    if (defaultSpeed == 0.2) { // only used in the use case of turning 1 degree, gives the motors time to actually move before stopping
        thread_sleep_for(10);
        motorStop();
    } else if (defaultSpeed == 0.5) { // otherwise stops immediately, had to be done in else if statement as otherwise it didn't work for unknown reasons
        motorStop();
    }
    if (abs(angle) != 1) {
        currentAngle = currentAngle + angle;
        currentAngle = currentAngle % 360;
        if (180 < currentAngle) {
            currentAngle = currentAngle - 360;
        }
        if (-180 >= currentAngle) {
            currentAngle = currentAngle + 360;
        }
    }
    Serial.println((String) "current angle: " + currentAngle);
    checkOrientation();
    tempAngle = -angle; // stores the negative of the inputted angle as when reversing the movements you'd want to want to reverse the turn too
    tempMili = 0;
    action = TURN;
    storeMovements();
    if (Exploring == false) {
        Serial.println("delay from explore false starts now");
        delay(1000);
    }
}

void Motor::turnLeft() {
    turnAngle(-90);
}
void Motor::turnRight() {
    turnAngle(90);
}
void Motor::turnAround() {
    turnAngle(180);
}
