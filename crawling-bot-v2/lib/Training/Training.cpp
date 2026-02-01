#include "Training.h"
#include "AHRS.h"
#include <Display.h>
// extern Display display;

// // #include "main.cpp"



// float Q_table[ANGLE_BINS][ANGLE_BINS][NUM_ACTIONS] = {0};

// Training::Training(ServoControl* left, ServoControl* right, AHRS* ahrs)
//     : leftServo(left),
//       rightServo(right),
//       ahrs(ahrs),
//       trainingActive(false),
//       modelLoaded(false)
// {
    
// }



// void Training::begin() {
//     Serial.println("Training module initialized (empty - to be implemented)");
// }

// // void Training::startTraining() {
// //     // TODO: Implement training logic
    

// //     Serial.println("Training started (implementation pending)");
// //     trainingActive = true;
// //     State s = getStateFromAngles(
// //         leftServo->getCurrentUpAngle(),
// //         rightServo->getCurrentUpAngle()
// //     );

// //     while (trainingActive)
// //     {
// //         Action a = selectAction(s); 
// //         executeAction(a);  
             
// //         AHRS::MovementSnapshot m = ahrs->getMeasurement();
// //         float r = calculateReward(m);
// //         State s_next = getStateFromAngles(
// //         leftServo->getCurrentUpAngle(),
// //         rightServo->getCurrentDownAngle()
// //         );
// //         updateQ(s, a, r, s_next);     
// //         s = s_next;

// //     }

// // }
// void Training::startTraining()
// {
//     Serial.println("Training started");
//     trainingActive = true;

//     State s = getStateFromAngles(
//         leftServo->getCurrentUpAngle(),
//         rightServo->getCurrentDownAngle()
//     );

//     uint32_t step = 0;
//     unsigned long lastLcd = 0;

//     while (trainingActive)
//     {
//         ahrs->resetMeasurement();

//         Action a = selectAction(s);
//         executeAction(a);

//         unsigned long t0 = millis();
//         while (millis() - t0 < 250) {
//             ahrs->update();
//             delay(5);
//         }

//         AHRS::MovementSnapshot m = ahrs->getMeasurement();
//         float r = calculateReward(m);


//         State s_next = getStateFromAngles(
//             leftServo->getCurrentUpAngle(),
//             rightServo->getCurrentDownAngle()
//         );

//         updateQ(s, a, r, s_next);
//         s = s_next;


//         if (millis() - lastLcd > 200)  
//         {
//             lastLcd = millis();

//             int upAngle   = leftServo->getCurrentUpAngle();
//             int downAngle = rightServo->getCurrentDownAngle();

//             display.clear();

//             display.setCursor(0, 0);
//             display.print("Step:");
//             display.print((int)step);

//             display.setCursor(0, 16);
//             display.print("R:");
//             display.print(String(r, 2)); 

//             display.setCursor(0, 32);
//             display.print("D:");
//             display.print(String(m.deltaDistance, 2));

//             display.setCursor(0, 48);
//             display.print("U:");
//             display.print(upAngle);
//             display.print(" D:");
//             display.print(downAngle);

//             display.refresh();
//         }

//         step++;
//         delay(1);
//     }
// }

// void Training::stopTraining() {
//     // TODO: Implement training stop logic
//     Serial.println("Training stopped");
//     trainingActive = false;
//     modelLoaded = true;//replace
// }

// bool Training::isTraining() {
//     return trainingActive;
// }

// void Training::executeLearnedBehavior() {
//     // TODO: Implement learned behavior execution
//     Serial.println("Executing learned behavior (implementation pending)");
//     State s = getStateFromAngles(
//         leftServo->getCurrentUpAngle(),
//         rightServo->getCurrentDownAngle()
//     );
//     float bestQ = Q_table[s.left][s.right][0];
//     Action best = (Action)0;

//     for (int a = 1; a < NUM_ACTIONS; a++)
//     {
//         if (Q_table[s.left][s.right][a] > bestQ)
//         {
//             bestQ = Q_table[s.left][s.right][a];
//             best = (Action)a;
//         }
//     }
//     Action a = best;
//     executeAction(a);
// }

// bool Training::hasLearnedBehavior() {
//     // TODO: Implement check for learned behavior
//     return modelLoaded;
// }

// void Training::saveModel() {
//     // TODO: Implement model saving
//     Serial.println("Saving model (implementation pending)");
// }

// void Training::loadModel() {
//     // TODO: Implement model loading
//     Serial.println("Loading model (implementation pending)");
//     modelLoaded = false;
// }
// void Training::resetModel()
// {
//     for (int i = 0; i < ANGLE_BINS; i++)
//     {
//         for (int j = 0; j < ANGLE_BINS; j++)
//         {
//             for (int a = 0; a < NUM_ACTIONS; a++)
//             {
//                 Q_table[i][j][a] = 0.0;
//             }
//         }
//     }

//     Serial.println("Resetting model (implementation pending)");
//     modelLoaded = false;
// }



// int Training::discretizeAngle(int angle)
// {
//     angle = constrain(angle, 0, 180);//180

//     int bin = (angle * ANGLE_BINS) / 181 ;//181  
//     bin = constrain(bin, 0, ANGLE_BINS - 1);
//     return bin;
// }

    
// Training::State Training::getStateFromAngles(int leftAngle, int rightAngle)
// {
//     State s;
//     s.left  = discretizeAngle(leftAngle);
//     s.right = discretizeAngle(rightAngle);
//     return s;
// }

// float Training::calculateReward(const AHRS::MovementSnapshot& m)
// {
//     const float forward_direction = 1.0;   //0.5
//     const float backward_direction = 0.2;

//     if (m.deltaDistance > forward_direction)
//         return +2.0;    //1    
//     else if (m.deltaDistance < backward_direction)
//         return -2.0;       //-1  
//     else
//         return -0.5;         
// }

// // Q_table[state_left][state_right][action] = value;


// Training::Action Training::selectAction(State s){
//     const float epsilon = 0.1; 
//     if (random(100) < (epsilon * 100)) {
//         return static_cast<Action>(random(NUM_ACTIONS));
//     } else {
//         float maxQ = Q_table[s.left][s.right][0];
//         Action bestAction = static_cast<Action>(0);

//         for (int a = 1; a < NUM_ACTIONS; a++) {
//             if (Q_table[s.left][s.right][a] > maxQ) {
//                 maxQ = Q_table[s.left][s.right][a];
//                 bestAction = static_cast<Action>(a);
//             }
//         }
//         return bestAction;
//     }
// }

// // void Training::executeAction(Action a)
// // {
// //     const int STEP_ANGLE = 10;  //20
// //     switch (a) {
// //         case BOTH_UP:
// //             leftServo->moveUpSmooth(leftServo->getCurrentUpAngle() + STEP_ANGLE);
// //             rightServo->moveDownSmooth(rightServo->getCurrentDownAngle() + STEP_ANGLE);
// //             break;
// //         case BOTH_DOWN:
// //             leftServo->moveUpSmooth(leftServo->getCurrentDownAngle() - STEP_ANGLE);
// //             rightServo->moveDownSmooth(rightServo->getCurrentDownAngle() - STEP_ANGLE);
// //             break;
// //         case LEFT_UP_RIGHT_DOWN:
// //             leftServo->moveUpSmooth(leftServo->getCurrentUpAngle() + STEP_ANGLE);
// //             rightServo->moveDownSmooth(rightServo->getCurrentDownAngle() - STEP_ANGLE);
// //             break;
// //         case LEFT_DOWN_RIGHT_UP:
// //             leftServo->moveUpSmooth(leftServo->getCurrentDownAngle() - STEP_ANGLE);
// //             rightServo->moveDownSmooth(rightServo->getCurrentUpAngle() + STEP_ANGLE);
// //             break;
// //     }
// //     delay(200); 
// // }

// void Training::executeAction(Action a)
// {
//     const int STEP_ANGLE = 20;

//     int up   = leftServo->getCurrentUpAngle();        
//     int down = rightServo->getCurrentDownAngle();     

//     int upTarget = up;
//     int downTarget = down;

//     switch (a)
//     {
//         case BOTH_UP:
//             upTarget   = up   + STEP_ANGLE;
//             downTarget = down + STEP_ANGLE;
//             break;

//         case BOTH_DOWN:
//             upTarget   = up   - STEP_ANGLE;
//             downTarget = down - STEP_ANGLE;
//             break;

//         case LEFT_UP_RIGHT_DOWN:
//             upTarget   = up   + STEP_ANGLE;
//             downTarget = down - STEP_ANGLE;
//             break;

//         case LEFT_DOWN_RIGHT_UP:
//             upTarget   = up   - STEP_ANGLE;
//             downTarget = down + STEP_ANGLE;
//             break;
//     }

//     upTarget   = constrain(upTarget, 0, 180);
//     downTarget = constrain(downTarget, 0, 180);

//     // Display display;
//     // display.clear();
//     // display.print(downTarget , 0 ,0);
//     // display.refresh();
//     // delay(200);
//     // display.clear();
//     // display.print(upTarget , 0 ,0);
//     // display.refresh();
//     // delay(200);

//     leftServo->moveUpSmooth(upTarget);
//     rightServo->moveDownSmooth(downTarget);

//     delay(200);
// }













// void Training::updateQ(State s, Action a, float r, State s_next)
// {
//     const float alpha = 0.1; 
//     const float gamma = 0.9;  

   
//     float maxQ_next = Q_table[s_next.left][s_next.right][0];
//     for (int action = 1; action < NUM_ACTIONS; action++) {
//         if (Q_table[s_next.left][s_next.right][action] > maxQ_next) {
//             maxQ_next = Q_table[s_next.left][s_next.right][action];
//         }
//     }

//     if (fabs(alpha * (r + gamma * maxQ_next - Q_table[s.left][s.right][a])) < 0.001) {
//         stopTraining();
//     }

//     Q_table[s.left][s.right][a] += alpha * (r + gamma * maxQ_next - Q_table[s.left][s.right][a]);


// }


// // /////////////////////////// third /////////////////////////////

// // #include "Training.h"

// // #include <math.h>

// // // Q_table[up_bin][down_bin][action]
// // static float Q_table[ANGLE_BINS][ANGLE_BINS][NUM_ACTIONS] = {0};

// // Training::Training(ServoControl *servos, AHRS *ahrs)
// //     : trainingActive(false),
// //       modelLoaded(false),
// //       servos(servos),
// //       ahrs(ahrs)
// // {
// // }

// // void Training::begin()
// // {
// //     // اگر seed ندهیم، randomها ممکن است تکراری شوند
// //     randomSeed((uint32_t)micros());
// //     Serial.println("Training module initialized");
// // }

// // void Training::startTraining()
// // {
// //     if (!servos || !ahrs)
// //     {
// //         Serial.println("Training start failed: servos/ahrs is null");
// //         return;
// //     }

// //     Serial.println("Training started");
// //     trainingActive = true;
// //     modelLoaded = false;

// //     ahrs->resetDisplacement();
// //     ahrs->resetMeasurement();

// //     State s = getStateFromAngles(
// //         servos->getCurrentUpAngle(),
// //         servos->getCurrentDownAngle());

// //     const uint32_t MAX_STEPS   = 800;
// //     const uint32_t STALE_LIMIT = 120;
// //     uint32_t staleCount = 0;

// //     for (uint32_t step = 0; trainingActive && step < MAX_STEPS; ++step)
// //     {
// //         ahrs->resetMeasurement();

// //         Action a = selectAction(s);
// //         executeAction(a);

// //         const uint32_t MEASURE_MS = 280;
// //         uint32_t t0 = millis();
// //         while (millis() - t0 < MEASURE_MS)
// //         {
// //             ahrs->update();
// //             delay(5);
// //         }

// //         AHRS::MovementSnapshot m = ahrs->getMeasurement();
// //         float r = calculateReward(m);

// //         State s_next = getStateFromAngles(
// //             servos->getCurrentUpAngle(),
// //             servos->getCurrentDownAngle());

// //         float absUpdate = updateQ(s, a, r, s_next);

// //         if (absUpdate < 0.001f)
// //             staleCount++;
// //         else
// //             staleCount = 0;

// //         if (staleCount >= STALE_LIMIT)
// //         {
// //             Serial.println("Training stopped: stale updates");
// //             stopTraining();
// //             break;
// //         }

// //         s = s_next;

// //         delay(1);
// //     }

// //     if (trainingActive)
// //     {
// //         Serial.println("Training finished: max steps reached");
// //         stopTraining();
// //     }
// // }

// // void Training::stopTraining()
// // {
// //     trainingActive = false;
// //     modelLoaded = true; 
// //     Serial.println("Training stopped");
// // }

// // bool Training::isTraining()
// // {
// //     return trainingActive;
// // }

// // void Training::executeLearnedBehavior()
// // {
// //     if (!modelLoaded)
// //         return;

// //     State s = getStateFromAngles(
// //         servos->getCurrentUpAngle(),
// //         servos->getCurrentDownAngle());

// //     float bestQ = Q_table[s.up][s.down][0];
// //     Action best = (Action)0;

// //     for (int a = 1; a < NUM_ACTIONS; a++)
// //     {
// //         if (Q_table[s.up][s.down][a] > bestQ)
// //         {
// //             bestQ = Q_table[s.up][s.down][a];
// //             best = (Action)a;
// //         }
// //     }

// //     executeAction(best);
// // }

// // bool Training::hasLearnedBehavior()
// // {
// //     return modelLoaded;
// // }

// // void Training::saveModel()
// // {
// //     Serial.println("Saving model (not implemented yet)");
// // }

// // void Training::loadModel()
// // {
// //     Serial.println("Loading model (not implemented yet)");
// //     modelLoaded = false;
// // }

// // void Training::resetModel()
// // {
// //     for (int i = 0; i < ANGLE_BINS; i++)
// //     {
// //         for (int j = 0; j < ANGLE_BINS; j++)
// //         {
// //             for (int a = 0; a < NUM_ACTIONS; a++)
// //             {
// //                 Q_table[i][j][a] = 0.0f;
// //             }
// //         }
// //     }
// //     Serial.println("Model reset");
// //     modelLoaded = false;
// // }

// // int Training::discretizeAngle(int angle)
// // {
// //     angle = constrain(angle, 0, 180);

// //     int bin = (angle * ANGLE_BINS) / 181;
// //     bin = constrain(bin, 0, ANGLE_BINS - 1);
// //     return bin;
// // }

// // Training::State Training::getStateFromAngles(int upAngle, int downAngle)
// // {
// //     State s;
// //     s.up = (uint8_t)discretizeAngle(upAngle);
// //     s.down = (uint8_t)discretizeAngle(downAngle);
// //     return s;
// // }

// // float Training::calculateReward(const AHRS::MovementSnapshot &m)
// // {
    

// //     const float goodMoveCm = 0.5f;
// //     const float tinyMoveCm = 0.15f;

// //     if (m.deltaDistance >= goodMoveCm)
// //         return +1.0f;
// //     if (m.deltaDistance <= tinyMoveCm)
// //         return -1.0f;
// //     return -0.2f;
// // }

// // Training::Action Training::selectAction(State s)
// // {
// //     const float epsilon = 0.12f;

// //     if (random(1000) < (int)(epsilon * 1000))
// //     {
// //         return (Action)random(NUM_ACTIONS);
// //     }

// //     float maxQ = Q_table[s.up][s.down][0];
// //     for (int a = 1; a < NUM_ACTIONS; a++)
// //     {
// //         float q = Q_table[s.up][s.down][a];
// //         if (q > maxQ)
// //             maxQ = q;
// //     }

// //     int candidates[NUM_ACTIONS];
// //     int count = 0;
// //     const float TOL = 1e-6f;
// //     for (int a = 0; a < NUM_ACTIONS; a++)
// //     {
// //         if (fabsf(Q_table[s.up][s.down][a] - maxQ) <= TOL)
// //             candidates[count++] = a;
// //     }

// //     int pick = candidates[random(count)];
// //     return (Action)pick;
// // }

// // void Training::executeAction(Action a)
// // {

// //     const int STEP_ANGLE = 20;

// //     int up = servos->getCurrentUpAngle();
// //     int down = servos->getCurrentDownAngle();

// //     int upTarget = up;
// //     int downTarget = down;

// //     switch (a)
// //     {
// //     case BOTH_UP:
// //         upTarget = up + STEP_ANGLE;
// //         downTarget = down + STEP_ANGLE;
// //         break;
// //     case BOTH_DOWN:
// //         upTarget = up - STEP_ANGLE;
// //         downTarget = down - STEP_ANGLE;
// //         break;
// //     case LEFT_UP_RIGHT_DOWN:
// //         upTarget = up + STEP_ANGLE;
// //         downTarget = down - STEP_ANGLE;
// //         break;
// //     case LEFT_DOWN_RIGHT_UP:
// //         upTarget = up - STEP_ANGLE;
// //         downTarget = down + STEP_ANGLE;
// //         break;
// //     }

// //     upTarget = constrain(upTarget, 0, 180);
// //     downTarget = constrain(downTarget, 0, 180);

// //     servos->moveUpSmooth(upTarget, 8);
// //     servos->moveDownSmooth(downTarget, 8);

// //     delay(80);
// // }

// // float Training::updateQ(State s, Action a, float r, State s_next)
// // {
// //     const float alpha = 0.12f;
// //     const float gamma = 0.90f;

// //     float maxQ_next = Q_table[s_next.up][s_next.down][0];
// //     for (int action = 1; action < NUM_ACTIONS; action++)
// //     {
// //         float q = Q_table[s_next.up][s_next.down][action];
// //         if (q > maxQ_next)
// //             maxQ_next = q;
// //     }

// //     float &qsa = Q_table[s.up][s.down][(int)a];
// //     float td = (r + gamma * maxQ_next) - qsa;
// //     float update = alpha * td;
// //     qsa += update;

// //     return fabsf(update);
// // }

// Training.cpp






///////////////////////////////////////////// forth condition ////////////////////////////////////////////

#include "Training.h"
#include "AHRS.h"

float Q_table[ANGLE_BINS][ANGLE_BINS][NUM_ACTIONS] = {0};

Training::Training(ServoControl* left, ServoControl* right, AHRS* ahrs)
    : leftServo(left),
      rightServo(right),
      ahrs(ahrs),
      trainingActive(false),
      modelLoaded(false)
{
    
}



void Training::begin() {
    Serial.println("Training module initialized (empty - to be implemented)");
}

void Training::startTraining() {
    // TODO: Implement training logic
    

    Serial.println("Training started (implementation pending)");
    trainingActive = true;
    State s = getStateFromAngles(
        leftServo->getCurrentUpAngle(),
        rightServo->getCurrentUpAngle()
    );

    while (trainingActive)
    {
        Action a = selectAction(s); 
        executeAction(a);           
        AHRS::MovementSnapshot m = ahrs->getMeasurement();
        float r = calculateReward(m);
        State s_next = getStateFromAngles(
        leftServo->getCurrentUpAngle(),
        rightServo->getCurrentDownAngle()
        );
        updateQ(s, a, r, s_next);     
        s = s_next;

    }

}

void Training::stopTraining() {
    // TODO: Implement training stop logic
    Serial.println("Training stopped");
    trainingActive = false;
}

bool Training::isTraining() {
    return trainingActive;
}

void Training::executeLearnedBehavior() {
    // TODO: Implement learned behavior execution
    Serial.println("Executing learned behavior (implementation pending)");
    State s = getStateFromAngles(
        leftServo->getCurrentUpAngle(),
        rightServo->getCurrentDownAngle()
    );
    float bestQ = Q_table[s.left][s.right][0];
    Action best = (Action)0;

    for (int a = 1; a < NUM_ACTIONS; a++)
    {
        if (Q_table[s.left][s.right][a] > bestQ)
        {
            bestQ = Q_table[s.left][s.right][a];
            best = (Action)a;
        }
    }
    Action a = best;
    executeAction(a);
}

bool Training::hasLearnedBehavior() {
    // TODO: Implement check for learned behavior
    return modelLoaded;
}

void Training::saveModel() {
    // TODO: Implement model saving
    Serial.println("Saving model (implementation pending)");
}

void Training::loadModel() {
    // TODO: Implement model loading
    Serial.println("Loading model (implementation pending)");
    modelLoaded = false;
}
void Training::resetModel()
{
    for (int i = 0; i < ANGLE_BINS; i++)
    {
        for (int j = 0; j < ANGLE_BINS; j++)
        {
            for (int a = 0; a < NUM_ACTIONS; a++)
            {
                Q_table[i][j][a] = 0.0;
            }
        }
    }

    Serial.println("Resetting model (implementation pending)");
    modelLoaded = false;
}



int Training::discretizeAngle(int angle)
{
  angle = constrain(angle, 0, 180);

  int bin = (angle * ANGLE_BINS) / 181;  
  bin = constrain(bin, 0, ANGLE_BINS - 1);
  return bin;
}

    
Training::State Training::getStateFromAngles(int leftAngle, int rightAngle)
{
    State s;
    s.left  = discretizeAngle(leftAngle);
    s.right = discretizeAngle(rightAngle);
    return s;
}

float Training::calculateReward(const AHRS::MovementSnapshot& m)
{
    const float forward_direction = 0.5;   
    const float backward_direction = 0.2;

    if (m.deltaDistance > forward_direction)
        return +1.0;        
    else if (m.deltaDistance < backward_direction)
        return -1.0;         
    else
        return -0.5;         
}

// Q_table[state_left][state_right][action] = value;


Training::Action Training::selectAction(State s){
    const float epsilon = 0.1; 
    if (random(100) < (epsilon * 100)) {
        return static_cast<Action>(random(NUM_ACTIONS));
    } else {
        float maxQ = Q_table[s.left][s.right][0];
        Action bestAction = static_cast<Action>(0);

        for (int a = 1; a < NUM_ACTIONS; a++) {
            if (Q_table[s.left][s.right][a] > maxQ) {
                maxQ = Q_table[s.left][s.right][a];
                bestAction = static_cast<Action>(a);
            }
        }
        return bestAction;
    }
}

void Training::executeAction(Action a)
{
    const int STEP_ANGLE = 20;  
    switch (a) {
        case BOTH_UP:
            leftServo->moveUpSmooth(leftServo->getCurrentUpAngle() + STEP_ANGLE);
            rightServo->moveDownSmooth(rightServo->getCurrentDownAngle() + STEP_ANGLE);
            break;
        case BOTH_DOWN:
            leftServo->moveUpSmooth(leftServo->getCurrentDownAngle() - STEP_ANGLE);
            rightServo->moveDownSmooth(rightServo->getCurrentDownAngle() - STEP_ANGLE);
            break;
        case LEFT_UP_RIGHT_DOWN:
            leftServo->moveUpSmooth(leftServo->getCurrentUpAngle() + STEP_ANGLE);
            rightServo->moveDownSmooth(rightServo->getCurrentDownAngle() - STEP_ANGLE);
            break;
        case LEFT_DOWN_RIGHT_UP:
            leftServo->moveUpSmooth(leftServo->getCurrentDownAngle() - STEP_ANGLE);
            rightServo->moveDownSmooth(rightServo->getCurrentUpAngle() + STEP_ANGLE);
            break;
    }
    delay(200); 
}

void Training::updateQ(State s, Action a, float r, State s_next)
{
    const float alpha = 0.1; 
    const float gamma = 0.9;  

   
    float maxQ_next = Q_table[s_next.left][s_next.right][0];
    for (int action = 1; action < NUM_ACTIONS; action++) {
        if (Q_table[s_next.left][s_next.right][action] > maxQ_next) {
            maxQ_next = Q_table[s_next.left][s_next.right][action];
        }
    }

    if (fabs(alpha * (r + gamma * maxQ_next - Q_table[s.left][s.right][a])) < 0.001) {
        stopTraining();
    }

    Q_table[s.left][s.right][a] += alpha * (r + gamma * maxQ_next - Q_table[s.left][s.right][a]);


}


// /////////////////////////// third /////////////////////////////

#include "Training.h"

#include <math.h>

// Q_table[up_bin][down_bin][action]
static float Q_table[ANGLE_BINS][ANGLE_BINS][NUM_ACTIONS] = {0};

Training::Training(ServoControl *servos, AHRS *ahrs)
    : trainingActive(false),
      modelLoaded(false),
      servos(servos),
      ahrs(ahrs)
{
}

void Training::begin()
{
    // اگر seed ندهیم، randomها ممکن است تکراری شوند
    randomSeed((uint32_t)micros());
    Serial.println("Training module initialized");
}

void Training::startTraining()
{
    if (!servos || !ahrs)
    {
        Serial.println("Training start failed: servos/ahrs is null");
        return;
    }

    Serial.println("Training started");
    trainingActive = true;
    modelLoaded = false;

    ahrs->resetDisplacement();
    ahrs->resetMeasurement();

    State s = getStateFromAngles(
        servos->getCurrentUpAngle(),
        servos->getCurrentDownAngle());

    const uint32_t MAX_STEPS   = 800;
    const uint32_t STALE_LIMIT = 120;
    uint32_t staleCount = 0;

    for (uint32_t step = 0; trainingActive && step < MAX_STEPS; ++step)
    {
        ahrs->resetMeasurement();

        Action a = selectAction(s);
        executeAction(a);

        const uint32_t MEASURE_MS = 280;
        uint32_t t0 = millis();
        while (millis() - t0 < MEASURE_MS)
        {
            ahrs->update();
            delay(5);
        }

        AHRS::MovementSnapshot m = ahrs->getMeasurement();
        float r = calculateReward(m);

        State s_next = getStateFromAngles(
            servos->getCurrentUpAngle(),
            servos->getCurrentDownAngle());

        float absUpdate = updateQ(s, a, r, s_next);

        if (absUpdate < 0.001f)
            staleCount++;
        else
            staleCount = 0;

        if (staleCount >= STALE_LIMIT)
        {
            Serial.println("Training stopped: stale updates");
            stopTraining();
            break;
        }

        s = s_next;

        delay(1);
    }

    if (trainingActive)
    {
        Serial.println("Training finished: max steps reached");
        stopTraining();
    }
}

void Training::stopTraining()
{
    trainingActive = false;
    modelLoaded = true; 
    Serial.println("Training stopped");
}

bool Training::isTraining()
{
    return trainingActive;
}

void Training::executeLearnedBehavior()
{
    if (!modelLoaded)
        return;

    State s = getStateFromAngles(
        servos->getCurrentUpAngle(),
        servos->getCurrentDownAngle());

    float bestQ = Q_table[s.up][s.down][0];
    Action best = (Action)0;

    for (int a = 1; a < NUM_ACTIONS; a++)
    {
        if (Q_table[s.up][s.down][a] > bestQ)
        {
            bestQ = Q_table[s.up][s.down][a];
            best = (Action)a;
        }
    }

    executeAction(best);
}

bool Training::hasLearnedBehavior()
{
    return modelLoaded;
}

void Training::saveModel()
{
    Serial.println("Saving model (not implemented yet)");
}

void Training::loadModel()
{
    Serial.println("Loading model (not implemented yet)");
    modelLoaded = false;
}

void Training::resetModel()
{
    for (int i = 0; i < ANGLE_BINS; i++)
    {
        for (int j = 0; j < ANGLE_BINS; j++)
        {
            for (int a = 0; a < NUM_ACTIONS; a++)
            {
                Q_table[i][j][a] = 0.0f;
            }
        }
    }
    Serial.println("Model reset");
    modelLoaded = false;
}

int Training::discretizeAngle(int angle)
{
    angle = constrain(angle, 0, 180);

    int bin = (angle * ANGLE_BINS) / 181;
    bin = constrain(bin, 0, ANGLE_BINS - 1);
    return bin;
}

Training::State Training::getStateFromAngles(int upAngle, int downAngle)
{
    State s;
    s.up = (uint8_t)discretizeAngle(upAngle);
    s.down = (uint8_t)discretizeAngle(downAngle);
    return s;
}

float Training::calculateReward(const AHRS::MovementSnapshot &m)
{
    

    const float goodMoveCm = 0.5f;
    const float tinyMoveCm = 0.15f;

    if (m.deltaDistance >= goodMoveCm)
        return +1.0f;
    if (m.deltaDistance <= tinyMoveCm)
        return -1.0f;
    return -0.2f;
}

Training::Action Training::selectAction(State s)
{
    const float epsilon = 0.12f;

    if (random(1000) < (int)(epsilon * 1000))
    {
        return (Action)random(NUM_ACTIONS);
    }

    float maxQ = Q_table[s.up][s.down][0];
    for (int a = 1; a < NUM_ACTIONS; a++)
    {
        float q = Q_table[s.up][s.down][a];
        if (q > maxQ)
            maxQ = q;
    }

    int candidates[NUM_ACTIONS];
    int count = 0;
    const float TOL = 1e-6f;
    for (int a = 0; a < NUM_ACTIONS; a++)
    {
        if (fabsf(Q_table[s.up][s.down][a] - maxQ) <= TOL)
            candidates[count++] = a;
    }

    int pick = candidates[random(count)];
    return (Action)pick;
}

void Training::executeAction(Action a)
{

    const int STEP_ANGLE = 20;

    int up = servos->getCurrentUpAngle();
    int down = servos->getCurrentDownAngle();

    int upTarget = up;
    int downTarget = down;

    switch (a)
    {
    case BOTH_UP:
        upTarget = up + STEP_ANGLE;
        downTarget = down + STEP_ANGLE;
        break;
    case BOTH_DOWN:
        upTarget = up - STEP_ANGLE;
        downTarget = down - STEP_ANGLE;
        break;
    case LEFT_UP_RIGHT_DOWN:
        upTarget = up + STEP_ANGLE;
        downTarget = down - STEP_ANGLE;
        break;
    case LEFT_DOWN_RIGHT_UP:
        upTarget = up - STEP_ANGLE;
        downTarget = down + STEP_ANGLE;
        break;
    }

    upTarget = constrain(upTarget, 0, 180);
    downTarget = constrain(downTarget, 0, 180);

    servos->moveUpSmooth(upTarget, 8);
    servos->moveDownSmooth(downTarget, 8);

    delay(80);
}

float Training::updateQ(State s, Action a, float r, State s_next)
{
    const float alpha = 0.12f;
    const float gamma = 0.90f;

    float maxQ_next = Q_table[s_next.up][s_next.down][0];
    for (int action = 1; action < NUM_ACTIONS; action++)
    {
        float q = Q_table[s_next.up][s_next.down][action];
        if (q > maxQ_next)
            maxQ_next = q;
    }

    float &qsa = Q_table[s.up][s.down][(int)a];
    float td = (r + gamma * maxQ_next) - qsa;
    float update = alpha * td;
    qsa += update;

    return fabsf(update);
}










