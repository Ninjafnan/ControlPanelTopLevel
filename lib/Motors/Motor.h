/**
 * library for the TB6612FNG Motor, the class can be split into 2 subsections,
 * the first initialising basic functions of the motors and encoders
 * which are used together in the to perform more complex actions such as turning and ramping speed
 * the 2nd section is a coordinate system and movement history section to 
 */

#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <mbed.h>
using namespace mbed;
#pragma once

class Motor {

public:
    /**
     * @brief Construct a new Motor object
     *
     * @param MotorADir Pin for motor A direction control
     * @param MotorBDir Pin for motor B direction control
     * @brief one of motor's has a flipped board which means the direction controls reversed
     * this means that whislst the motors mirror eachother
     * if both directions are set to 0 they will move in the same direction
     * as opposed to e.g a normal MotorADir = 0 and MotorBDir = 1 values resulting in a "forward" direction
     * No longer true due to having to switch out motors
     *
     * @param MotorAPWM Pin for motor A PWM speed control
     * @param MotorBPWM Pin for motor B PWM speed control
     */
    Motor(PinName MotorADir, PinName MotorBDir, PinName MotorAPWM,
          PinName MotorBPWM);

    /**
     * @brief Sets the direction and speed of the motors
     *
     * @param directionA Direction for motorA (e.g., 1 or 0)
     * @param speedA Speed for motorA (0.0f to 1.0f)
     * @param directionB Direction for motorB (e.g., 1 or 0)
     * @param speedB Speed for motorB (0.0f to 1.0f)
     */

    void moveMotors(int directionA, float speedA, int directionB, float speedB);

    /**
     * @brief Sets up the PWM for both motors
     */
    void bothMotorSetup();

    /**
     * @brief Sets the speed of both motors to 0
     */
    void motorStop();

    /**
     * @brief calculates the how many times a Motor/wheel has done a revolution depending on it's Encoder count
     */
    void revolutionsA();
    void revolutionsB();

    /**
     * @brief increments the encoder count depending on the direction of the motor
     * a positive increment means the motor is moving "forwards" whereas,
     * a negative increment means the motor is moving "backwards"
     */
    void countPulseA();
    void countPulseB();

    /**
     * @brief slowly increases the speed of both motors to a desired target value
     *
     * @param targetSpeed (0-1.0f)Sets the desired max speeds the motors will ramp too
     * @param type 1-4 responsible for which ways the motors will move in
     * 1: results in the robot moving forward
     * 2: results in the robot moving backwards
     * 3: results in the robot turning left
     * 4: results in the robot turning right
     */
    void rampSpeed(float targetSpeed, int type);

    /**
     * @brief moves the robot a desired amount in milimeters
     *
     * @param mili can be any negative or positive number
     * if negative the robot will move backwards that distance in mm
     * if positive the robot will move forwards in the distance in mm
     */
    void move(int mili);

    /**
     * @brief turns the robot a desired angle
     *
     * @param angle can be any negative or positive number
     * if negative the robot will move left/anticlockwise that angle by calculating
     * the amount of encoder ticks it would need to travel that far
     * if positive the robot will move right/clockwise the same way
     */
    void turnAngle(int angle);

    // methods that call the turnAngle with a preseset angle for simplicity
    void turnLeft();
    void turnRight();
    void turnAround();

    /**
     * @brief variable indiciative of current angle the robot is facing through tracking of turns
     *if 0 degrees the robot is facing north
     *if 90 it's facing east
     *if 180
     *if -90 it's facing west
     */
    int currentAngle = -90;

    /**
     * @param xCoordMode, yCoordMode will only ever have the values 0, 1 and -1 indicates the multiplicative value to times when adding to map
     */
    int xCoordMode = -1;
    int yCoordMode = 0;

    /**
     * @brief updates the coordinates of the robot within the maze
     */
    void updateCoords();

    /**
     * @param currentXCoord, currentYCoord value represents current X and Y coordinate in the maze
     */
    int currentXCoord = 0;
    int currentYCoord = 0;

    /**
     * @brief function changes the above x and coord mode to ensure accurate mapping
     */
    void checkOrientation();

    /**
     * @brief struct to store the type of movement and value inputted to said movement
     *
     * @param action represents the function called either the move() or the turnAngle() function
     *
     * @param value will be equivalent to the inputted value into either the mili or angle
     */
    struct aMove {
        int action;
        int value;
    };

    /** 
     * @brief array made out of the aMove struct, stores a history of made movements
     */
    aMove history[150] = {};

    /** 
     * @brief array made out of the aMove struct, would store the opposite of the history array with optimizations
     */
    aMove backToStart[150] = {};

    /**
     * @brief function stores the current movement in an array in the type of the "aMove" struct to reverse when it reaches the end of the maze
     */
    void storeMovements();

    /**
     * @brief function that takes the history of movements and attempts to remove redundancies
     */
    void optimiseMovements();

    /**
     * @brief function takes the stored list of moves in the history array and does them in reverse to get to the end
     */
    void playbackMovements();

    /**
     * @param historyCounter represents how many movements have been stored
     */
    int historyCounter = 0;

    /**
     * @param TURN represents an arbitary number exists to be assigned to check what most recent action was and to make code more readable
     */
    int TURN = 3333;
    /**
     * @param MOVE represents an arbitary number exists to be assigned to check what most recent action was and to make code more readable
     */
    int MOVE = 2222;
    /**
     * @param action value represents the action stored
     * if 3333: it's turnAngle action/function
     * if 2222: it's a movement action/function
     */
    int action;

    /**
     * @param Exploring
     * if true: robot is still solving the maze
     * if false: robot is making it's way back
     */
    bool Exploring = true;

private:
    DigitalOut MotorADir; // pin for Motor A direction control
    DigitalOut MotorBDir; // pin for Motor B direction control
    PwmOut MotorAPWM;     // pin for Motor A speed control
    PwmOut MotorBPWM;     // pin for Motor B speed control
    long int ShaftRevA;   // int for storing the amount of times motor A's has revolved
    long int ShaftRevB;   // int for storing the amount of times motor B's has revolved
    long int EncCountA;   // int for storing the encoder count of motor A
    long int EncCountB;   // int for storing the encoder count of motor B
    long int timer;       // int for to increment how many times the encoder increases in value
    int tempMili;         // temp variable equivalant to mili passed into the move function, tempMili is
                          // passed into updatecoords function to update the coordinate of the robot and 
                          // into the action value of the aMove struct when storing a move value

    int tempAngle; //passed into the the action value of the aMove struct when storing a turnAngle value

    /**
     * @brief calculated by 660(no of encoder rounts in a revolution) divided by
     * (2*pi*48.5mm) wheel diameter* number is slightly larger than what was measured
     * to accounts for the wheel's wobble/ it not spinning perfectly straight
     */
    const float ENC_TICK_PER_MM = 2.165819844;
    const float SCALE_FACTOR = 10.0; // constant to keep distances accurate
    /**
     * @brief calculated by 18cm distance between times by pi and then divided by
     * 360 to find the distance required for 1 degree of turning
     *
     * it is acknowledged that the true value should be 1.57.. and something went wrong in my intial calculation
     * but refactoring the code caused errors in precision even when accounting for previosuly added scale factors to correct it
     * so begrudgingly been kept this way despite my best efforts
     */
    const float ONE_DEGREE_IN_MM = 0.01570796327;
};

#endif