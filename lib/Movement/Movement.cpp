#include <Arduino.h>
#include <Movement.h>

// initialise encoder pins
InterruptIn EncA(P1_11);
InterruptIn EncB(P1_12);

// Timeout function exists to prevent robot from getting stuck in a move it can't complete
Timeout timeout;
volatile bool timeout_occurred = false;
void onTimeout() { timeout_occurred = true; }
void beginTimeout(float time) {
    timeout.detach();
    timeout.attach(&onTimeout, time);
}

Movement::Movement(PinName MotorADir, PinName MotorBDir, PinName MotorAPWM, PinName MotorBPWM)
    : MotorADir(MotorADir), MotorBDir(MotorBDir), MotorAPWM(MotorAPWM), MotorBPWM(MotorBPWM) {}

void Movement::moveMotors(int directionA, float speedA, int directionB, float speedB) {
    MotorADir = directionA;
    MotorAPWM.write(speedA);
    MotorBDir = directionB;
    MotorBPWM.write(speedB);
}

void Movement::bothMotorSetup() {
    MotorAPWM.period_ms(1);
    MotorBPWM.period_ms(1);
}

void Movement::motorStop() {
    MotorAPWM.write(0);
    MotorBPWM.write(0);
}

void Movement::revolutionsA() {
    if (EncCountA % 6 == 0) {
        if (EncCountA % 110 == 0) {
            ShaftRevA++;
        }
    }
}
void Movement::revolutionsB() {
    if (EncCountB % 6 == 0) {
        if (EncCountB % 110 == 0) {
            ShaftRevB++;
        }
    }
}
void Movement::countPulseA() {
    if (MotorADir == 0) {
        EncCountA++;
    } else {
        EncCountA--;
    }
    revolutionsA();
}
void Movement::countPulseB() {
    if (MotorBDir == 0) {
        EncCountB++;
    } else {
        EncCountB--;
    }
    revolutionsB();
}

void Movement::rampSpeed(float targetSpeed, int type) {
    const float RAMP_FACTOR = 0.0005;
    float currentSpeed = 0.00000; // initialises the starting to speed to be 0
    int wantedDirA;
    int wantedDirB;

    if (type == 1) { // ramp movement forwards
        wantedDirA = 1;
        wantedDirB = 0;
    } else if (type == 2) { // ramp movement backwards
        wantedDirA = 0;
        wantedDirB = 1;
    } else if (type == 3) { // ramp movement turning right
        wantedDirA = 1;
        wantedDirB = 1;
    } else if (type == 4) { // ramp movement turning left
        wantedDirA = 0;
        wantedDirB = 0;
    } else {
        return; // if incorrect type is given exit function
    }

    EncCountA = 0; // resets encoder values to increase realiablity by removing any potential builtup errors
    EncCountB = 0;
    beginTimeout(5); // starts a timeout timer to stop movement attempt where it has failed due runnning into a wall
    while (currentSpeed < targetSpeed) {
        currentSpeed = currentSpeed + RAMP_FACTOR;
        moveMotors(wantedDirA, currentSpeed, wantedDirB, currentSpeed);
        Serial.println((String) "Current speed: " + currentSpeed + " Loop: " + timer);
        EncA.rise(callback(this, &Movement::countPulseA));
        EncB.rise(callback(this, &Movement::countPulseB));
        // Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " Timer: " + timer);
        timer++;
        // below checks are there to ensure the motors don't desync
        if (abs(EncCountA) < abs(EncCountB)) {
            moveMotors(wantedDirA, currentSpeed, wantedDirB, 0);
        }
        if (abs(EncCountB) < abs(EncCountA)) {
            moveMotors(wantedDirA, 0, wantedDirB, currentSpeed);
        }
        if (abs(EncCountB) == abs(EncCountA)) {
            moveMotors(wantedDirA, currentSpeed, wantedDirB, currentSpeed);
        }
        if (timeout_occurred) {
            // robot has gotten stuck and so the movement has been forcefully terminated
            Serial.println("timeout success");
            timeout_occurred = false; // resets timeout function
            break;
        }
    }
    motorStop(); // stops motors
}

void Movement::move(int mili) {
    int wantedDirA = 1;
    int wantedDirB = 0;
    tempMili = mili; // stores parameter to be stored in movement history for playback

    if (mili < 0) {
        // if mili is less than 0 it should be going backwards, otherwise it's going forwards
        wantedDirA = 0;
        wantedDirB = 1;
    } else if (0 < mili) {
        wantedDirA = 1;
        wantedDirB = 0;
    } else {
        return; // if 0 or somehow anything else is inputted that isn't allowed do nothing and exit the function
    }
    beginTimeout(5); // starts a timeout timer to stop movement attempt where it has failed due runnning into a wall
    moveMotors(wantedDirA, 0.5, wantedDirB, 0.5);
    int TargetEncVal = mili * ENC_TICK_PER_MM * SCALE_FACTOR;
    Serial.println((String) "Target Encoder Value: " + TargetEncVal);
    EncCountA = 0; // resets encoder values to increase realiablity by removing any potential builtup errors
    EncCountB = 0;

    while ((abs(EncCountA) < abs(TargetEncVal)) || (abs(EncCountB) < abs(TargetEncVal))) {
        EncA.rise(callback(this, &Movement::countPulseA));
        EncB.rise(callback(this, &Movement::countPulseB));
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
        if (timeout_occurred) {
            // robot has gotten stuck and so the movement has been forcefully terminated
            Serial.println("timeout success");
            timeout_occurred = false; // resets timeout function
            break;
        }
    }
    motorStop();
    updateCoords();
    tempAngle = 0; // sets angle to nothing to ensure only movement is stored
    action = MOVE; // sets action to move for movement storage purposes
    storeMovements();
    if (Exploring == false) { // only occoures on playbackmovements as no delay caused issues
        Serial.println("delay from explore false starts now");
        delay(1000);
    }
}

void Movement::checkOrientation() {
    switch (currentAngle) {
    case (0): // facing north/forwards towards the end
        yCoordMode = 1;
        xCoordMode = 0;
        break;
    case (90): // facing east/right
        yCoordMode = 0;
        xCoordMode = 1;
        break;
    case (180): // facing south/backwards
        yCoordMode = -1;
        xCoordMode = 0;
        break;
    case (-90): // facing west/left
        yCoordMode = 0;
        xCoordMode = -1;
        break;
    }
    Serial.println((String) " xCoordMode: " + xCoordMode + " yCoordMode: " + yCoordMode);
}
void Movement::updateCoords() {
    currentXCoord = currentXCoord + (tempMili * xCoordMode * SCALE_FACTOR);
    currentYCoord = currentYCoord + (tempMili * yCoordMode * SCALE_FACTOR);
    Serial.println((String) "xCoordMode: " + xCoordMode + " yCoordMode: " + yCoordMode + " xCoord: " + currentXCoord + " yCoord: " + currentYCoord);
    if (currentYCoord > 1630) {   // robot would have reached the end
        digitalWrite(LEDG, HIGH); // turn off green LED
        digitalWrite(LEDB, LOW);  // turn on Blue LED to indicate it's time to go back through the maze using array of stored moves
        Exploring = false;
        playbackMovements();
    }
}

void Movement::storeMovements() {
    // creates struct to store each pair of movement options
    aMove movement;
    movement.action = action; // stores either turn or move passed set by their respective functions before this is called

    if (Exploring == true) { // checks if robot is still exploring if it is it goes ahead with storing
        if (tempMili == 0) { // checks that move ISN'T being stored
            if (abs(tempAngle) == 1) { //if the angle is 1 then it's wallaligning and should therefore be ignored
                return;
            }
            movement.value = tempAngle; // stores angle value

        } else if (tempAngle == 0) {   // checks that turn ISN'T being stored
            movement.value = tempMili; // stores movement value
        }
        history[historyCounter] = movement; // stores the struct in an array and increments it
        historyCounter++;
        Serial.println((String) "movement action: " + movement.action + " movement value: " + movement.value + " historyCounter: " + historyCounter);
        // prints out array used to check correct movement storage
    }
}

void Movement::optimiseMovements() {
    // not fully functional at time of submission but kept to complete later and for the attempt
    int tempVal;
    aMove optimisedMovement; // creates new struct to store the optimised movement
    int j = 0;               // integer to increment array the optimised movements will be stored in
    aMove prevMove;          // exists as a fall back to ensure that move is currently being optimised due to how leaving the while loop works
    int prevVal;             // holds previous value to make sure correct thing stored after while loop is left and the action being checked is no longer a movement

    for (int i = historyCounter - 1; i >= 0; i--) {
        aMove movement;
        tempVal = 0;

        movement = history[i];
        // checks if the movement action is a move
        // if it is then then it it adds the movement value to a running total so that can be added to one big move at the end

        while (movement.action == MOVE) { // checks if the movement action is a move
            movement = history[i];
            tempVal = tempVal + optimisedMovement.value;
            Serial.println((String) " i val before: " + i); // print statement for me to check if everything was working correctly

            delay(1000); // slight delay just for the purpose of debugging
            prevMove = history[i];
            prevVal = prevMove.value;
            Serial.println((String) " prevVal: " + prevVal + " i: " + i); // what preval currently is and what the I value is
            i--;
            Serial.println((String) " i val after: " + i + " temp value momento: " + tempVal); // check to see if running total is correct
        }
        optimisedMovement.value = prevVal; // stores prev to check what it is, this was to here for me to try and solve the incorrect storing
        optimisedMovement.action = MOVE;   // sets new action to move in the optimised struct
        Serial.println((String) "optmovement action: " + optimisedMovement.action + " optmovement value: " + movement.value + " i: " + i);

        backToStart[j] = optimisedMovement; // stores the optimised move in a new array
        j++;                                // increments position in array that is being written too

        // after this version of optimisation worked a copy of it was to be created and refactored for angles, whilst that is much less noticeable
        //  optimisation unless my robot loops on it self for some reason i still think it would be ideal
        //  would also need to figure out how to make wall alignment work in this case?
        //  it's likely that the entire function would get moved to main and then simply done there instead as is the current plan to include wall alignment
    }
}
// curently works, would be attempted to be playbacking the optimised movements instead of simply history
// in reverse if i had gotten that to work therefore this function may also have to move to main to allow for wall alignment

void Movement::playbackMovements() {
    Exploring = false; // stops robot from attempting to explore anymore
    turnAround();      // makes it turn around to face the start
    move(3);
    // loop goes through each section of the array and plays back the stored movements in reverse to make it back

    for (int i = historyCounter - 1; i >= 0; i--) {
        aMove movement = history[i];
        Serial.println((String) "movement action: " + movement.action + " movement value: " + movement.value + " historyCounter: " + i);
        // checks if the action stored is a move or a turn and then it would pass the appropriate parameter depending on which
        if (movement.action == MOVE) {
            move(movement.value);
        } else if (movement.action == TURN) {
            turnAngle(movement.value);
        }
    }
}

void Movement::turnAngle(int angle) {
    int wantedDirA = 1;
    int wantedDirB = 0;
    float defaultSpeed = 0.5;
    // removes redundancies in any input angle as there no point in moving more than 360 degrees
    // as it only would introduces more encoder errors
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
        // if angle is less than 0 it should be turning in the left direction,
        // left meaning left of the ultrasonic if it's facing away from you as the quadrant is defined in this way...
        wantedDirA = 1;
        wantedDirB = 1;
    } else if (0 < angle) { //...otherwise it would be turning right and so these are used
        wantedDirA = 0;
        wantedDirB = 0;
    }

    EncA.rise(callback(this, &Movement::countPulseA));
    EncB.rise(callback(this, &Movement::countPulseB));
    beginTimeout(5); // starts a timeout timer to stop movement attempt where it has failed
                     // should in theory be impossible due to circular design but here for
    //  Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " Timer: " + timer);
    timer++;

    // starts motors moving as desired
    moveMotors(wantedDirA, defaultSpeed, wantedDirB, defaultSpeed);
    // resets encoder values to increase realiablity by removing any potential builtup errors
    EncCountA = 0;
    EncCountB = 0;
    int distanceTravelledInTurn = angle * ONE_DEGREE_IN_MM * SCALE_FACTOR;       // calculate the target distance travel to travel
    int TargetEncVal = distanceTravelledInTurn * ENC_TICK_PER_MM * SCALE_FACTOR; // find equivalent encoder value for that distance
                                                                                 // absolute values used so it doesn't matter which way the motors are turning
    while ((abs(EncCountA) < abs(TargetEncVal)) || (abs(-EncCountB) < abs(TargetEncVal))) {

        EncA.rise(callback(this, &Movement::countPulseA));
        EncB.rise(callback(this, &Movement::countPulseB));
        // Serial.println((String) "EncoderA:" + EncCountA + " Revolutions A:" + ShaftRevA + " EncoderB:" + EncCountB + " Revolutions B:" + ShaftRevB + " target Enc:" + TargetEncVal + " Timer: " + timer);
        timer++;
        // syncs up encoders by momentarilly turning 1 of them off if the other goes ahead to ensure it spins on the spot
        if (abs(EncCountA) < abs(-EncCountB)) {
            moveMotors(wantedDirA, defaultSpeed, wantedDirB, 0);
        }
        if (abs(-EncCountB) < abs(EncCountA)) {
            moveMotors(wantedDirA, 0, wantedDirB, defaultSpeed);
        }
        if (abs(-EncCountB) == abs(EncCountA)) { // if encoders are synced they continue as desired
            moveMotors(wantedDirA, defaultSpeed, wantedDirB, defaultSpeed);
        }
        if (timeout_occurred) {
            // robot has gotten stuck and so the movement has been forcefully terminated
            Serial.println("timeout success");
            timeout_occurred = false; // resets timeout function
            break;
        }
    }
    // only used in the use case of turning 1 degree, gives the motors time to actually move before stopping
    if (defaultSpeed == 0.2) {
        thread_sleep_for(10);
        motorStop();
        // otherwise stops immediately, had to be done in else if statement as otherwise it didn't work for unknown reasons
    } else if (defaultSpeed == 0.5) {
        motorStop();
    }
    if (abs(angle) != 1) {
        // does the same thing as the angle optimisation above but this is done for the compass orientation
        // so wants to make sure it's not being called when wall Alignment is using this function
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
    tempMili = 0;       // ensures temp mili is 0 so correct value of movement is stored
    action = TURN;      // sets action to turn for movement storage purposes
    storeMovements();
    if (Exploring == false) { // only occoures on playbackmovements as no delay caused issues
        Serial.println("delay from explore false starts now");
        delay(1000);
    }
}

void Movement::turnLeft() {
    turnAngle(-90);
}
void Movement::turnRight() {
    turnAngle(90);
}
void Movement::turnAround() {
    turnAngle(180);
}
