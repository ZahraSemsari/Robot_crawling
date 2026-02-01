#ifndef TRAINING_H
#define TRAINING_H
#pragma once
#define ANGLE_BINS 6
#define NUM_ACTIONS 4

#include <Arduino.h>
#include <ServoControl.h>
#include "AHRS.h"


class Training {
public:



    struct State {
        uint8_t left; //this is up  
        uint8_t right; //this is down 
    };





    enum Action {
        BOTH_UP,
        BOTH_DOWN,
        LEFT_UP_RIGHT_DOWN,
        LEFT_DOWN_RIGHT_UP
    };



    Training(ServoControl *left, ServoControl *right, AHRS* ahrs);
    void begin();
    
    // Training methods - to be implemented
    void startTraining();
    void stopTraining();
    bool isTraining();
    
    // Learning methods - to be implemented
    void executeLearnedBehavior();
    bool hasLearnedBehavior();
    
    // Data management - to be implemented
    void saveModel();
    void loadModel();
    void resetModel();


    // State functions 
    State getStateFromAngles(int leftAngle, int rightAngle);
    int discretizeAngle(int angle);

    //reward
    float calculateReward(const AHRS::MovementSnapshot& m);

    //q-table
    Action selectAction(State s);
    void updateQ(State s, Action a, float r, State s_next);

    void executeAction(Action a);

private:
    bool trainingActive;
    bool modelLoaded;
    ServoControl* leftServo;
    ServoControl* rightServo;
    AHRS* ahrs;

};

#endif // TRAINING_H



// #ifndef TRAINING_H
// #define TRAINING_H
// #pragma once

// #include <Arduino.h>
// #include <ServoControl.h>
// #include "AHRS.h"

// #define ANGLE_BINS 6

// #define NUM_ACTIONS 4

// class Training
// {
// public:
//     struct State
//     {
//         uint8_t up;   
//         uint8_t down; 
//     };

//     // اکشن‌ها (تغییر زاویه بازوها)
//     enum Action
//     {
//         BOTH_UP,
//         BOTH_DOWN,
//         LEFT_UP_RIGHT_DOWN, 
//         LEFT_DOWN_RIGHT_UP 
//     };

//     Training(ServoControl *servos, AHRS *ahrs);
//     void begin();

//     void startTraining();
//     void stopTraining();
//     bool isTraining();

//     void executeLearnedBehavior();
//     bool hasLearnedBehavior();

//     void saveModel();
//     void loadModel();
//     void resetModel();

//     State getStateFromAngles(int upAngle, int downAngle);
//     int discretizeAngle(int angle);

//     float calculateReward(const AHRS::MovementSnapshot &m);

//     Action selectAction(State s);
//     float updateQ(State s, Action a, float r, State s_next);

//     void executeAction(Action a);

// private:
//     bool trainingActive;
//     bool modelLoaded;

//     ServoControl *servos;
//     AHRS *ahrs;
// };

// #endif 

