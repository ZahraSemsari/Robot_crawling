// #include <Arduino.h>
// #include <Display.h>
// #include <AHRS.h>
// #include <ServoControl.h>
// #include <Network.h>
// #include <Training.h>
// #include <HealthCheck.h>

// // Pin definitions
// const uint8_t SERVO_PIN_DOWN = 16; // P1 on board
// const uint8_t SERVO_PIN_UP = 15;   // P2 on board

// // Global objects
// Display display;
// AHRS ahrs;
// ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);
// Network *network; // Heap allocation needed for initialization order with display
// Training training;
// HealthCheck healthCheck(&display, &ahrs, &servoControl);

// void setup()
// {
//     Serial.begin(115200);
//     delay(1000);

//     // Initialize Display
//     display.begin();
//     display.clear();
//     display.print("RL Robot V2", 0, 0);
//     display.setCursor(0, 16);
//     display.print("Initializing...");
//     delay(1000);

//     // Initialize Network
//     network = new Network(&display);
//     network->begin();
//     network->startOTATask();

//     // Display robot info
//     display.clear();
//     display.print("Robot #", 0, 0);
//     display.print(network->getRobotNumber());
//     display.setCursor(0, 16);
//     display.print("Setup...");
//     delay(1000);

//     // Initialize AHRS
//     display.clear();
//     display.print("Init AHRS...", 0, 0);
//     if (ahrs.begin())
//     {
//         display.setCursor(0, 16);
//         display.print("AHRS OK");
//     }
//     else
//     {
//         display.setCursor(0, 16);
//         display.print("AHRS Failed!");
//     }

    
//     delay(1000);

//     ahrs.calibrateAccelGyro();  
//     // ahrs.calibrateMag(); 

//     // Initialize Servos
//     display.clear();
//     display.print("Init Servos...", 0, 0);
//     servoControl.begin();
//     display.setCursor(0, 16);
//     display.print("Servos OK");
//     delay(1000);

//     // Initialize Training
//     training.begin();

//     // Setup complete
//     display.clear();
//     display.print("Setup Complete", 0, 0);
//     delay(2000);

//     // Run health check
//     healthCheck.run();

//     // Reset measurement for interval tracking
//     ahrs.resetMeasurement();
// }


/////////////////////////////////////////////////////// First Version ////////////////////////////////////////
// // void loop()
// // {
// //     // Update AHRS
// //     ahrs.update();

// //     // ===== MODE 1: Real-time continuous display =====
// //     // Uncomment this section to show current instantaneous values
// //     /*
// //     display.clear();

// //     // Line 1: Speed (cm/s)
// //     display.setCursor(0, 0);
// //     display.print("Spd: ");
// //     display.print(ahrs.getSpeed() * 100, 1);
// //     display.print(" cm/s");

// //     // Line 2: Acceleration (m/s^2)
// //     display.setCursor(0, 12);
// //     display.print("Acc: ");
// //     display.print(ahrs.getAccelMagnitude(), 2);
// //     display.print(" m/s2");

// //     // Line 3: Displacement X (cm)
// //     display.setCursor(0, 24);
// //     display.print("X: ");
// //     display.print(ahrs.getDisplacementX() * 100, 1);
// //     display.print(" cm");

// //     // Line 4: Displacement Y (cm)
// //     display.setCursor(0, 36);
// //     display.print("Y: ");
// //     display.print(ahrs.getDisplacementY() * 100, 1);
// //     display.print(" cm");

// //     display.refresh();
// //     delay(1000);
// //     */

// //     // ===== MODE 2: Interval measurement (every 2 seconds) =====
// //     // This shows average movement parameters since last measurement
// //     static unsigned long lastMeasurement = 0;
// //     unsigned long currentTime = millis();

// //     if (currentTime - lastMeasurement >= 2000)
// //     { // Every 2 seconds
// //         lastMeasurement = currentTime;

// //         // Get measurement data
// //         AHRS::MovementSnapshot measurement = ahrs.getMeasurement();

// //         // Display the interval data
// //         display.clear();

// //         // Line 1: Distance moved in interval (cm)
// //         display.setCursor(0, 0);
// //         display.print("Dist: ");
// //         display.print(measurement.deltaDistance, 1);
// //         display.print(" cm");

// //         // Line 2: Average speed in interval (cm/s)
// //         display.setCursor(1, 0);
// //         display.print("Spd: ");
// //         display.print(measurement.avgSpeed, 1);
// //         display.print(" cm/s");

// //         // Line 3: Average acceleration in interval (m/s^2)
// //         display.setCursor(2, 0);
// //         display.print("Acc: ");
// //         display.print(measurement.avgAcceleration, 2);
// //         display.print(" m/s2");

// //         // Line 4: Time interval
// //         display.setCursor(3, 0);
// //         display.print("Time: ");
// //         display.print(measurement.deltaTime, 1);
// //         display.print(" s");

// //         display.refresh();

// //         // Reset for next measurement
// //         ahrs.resetMeasurement();

// //         // Print to serial for debugging
// //         Serial.print("Distance: ");
// //         Serial.print(measurement.deltaDistance);
// //         Serial.print(" cm, Speed: ");
// //         Serial.print(measurement.avgSpeed);
// //         Serial.print(" cm/s, Accel: ");
// //         Serial.print(measurement.avgAcceleration);
// //         Serial.println(" m/s2");
// //     }
// // ServoControl leftServo(LEFT_DOWN_PIN, LEFT_UP_PIN);
// //ServoControl rightServo(RIGHT_DOWN_PIN, RIGHT_UP_PIN);


// //     // TODO: Implement main loop logic
// //     // - Read sensors
// //     // - Process data
// //     // - Execute training if active
// //     // - Execute learned behavior
// //     // - Control servos
// // }


// void loop()
// {
//     static bool phase1Done = false;
//     if (phase1Done)
//         return;

//     display.clear();
//     display.print("Hello World", 0, 0);
//     display.refresh();
//     delay(1000);

//     display.clear();
//     display.print("Bye Bye", 0, 0);
//     display.refresh();
//     delay(500);

//     // servoControl.moveUpSmooth(180, 10);
//     // delay(200);

//     // servoControl.moveUpSmooth(60, 10);
//     // delay(200);

//     // servoControl.moveUpSmooth(180, 10);
//     // delay(200);

//     // servoControl.moveUpSmooth(60, 10);
//     // delay(200);

//     // for the down servo
//     servoControl.moveDownSmooth(60);
//     delay(300);
    
//     servoControl.moveDownSmooth(120, 10);
//     delay(200);

//     servoControl.moveDownSmooth(60, 10);
//     delay(200);

//     servoControl.moveDownSmooth(120, 10);
//     delay(200);

//     servoControl.moveDownSmooth(60, 10);
//     delay(200);


//     servoControl.moveDown(120);
//     delay(500);

//     servoControl.setInitialPosition();

//     display.clear();
//     display.refresh();

//     phase1Done = true;
// }


// //////////////////////////////////////// Second Version ////////////////////////////////////////

#include <Arduino.h>

#include <Display.h>
#include <AHRS.h>
#include <ServoControl.h>
#include <Network.h>
#include <Training.h>
#include <HealthCheck.h>

const uint8_t SERVO_PIN_DOWN = 16; 
const uint8_t SERVO_PIN_UP   = 15; 


Display display;
AHRS ahrs;

ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);

Network* network;


Training training(&servoControl, &servoControl, &ahrs);
// Training training(&servoControl, &ahrs);
HealthCheck healthCheck(&display, &ahrs, &servoControl);

// متغیرهای کمکی برای محاسبه بازه‌ای (Interval Calculation)
float lastPosX = 0, lastPosY = 0, lastPosZ = 0;
unsigned long lastLogTime = 0;

void setup()
{
    Serial.begin(115200);

    display.begin();
    delay(1000);
    display.clear();
    display.print("RL Robot V2", 0, 0);
    display.setCursor(0, 16);
    display.print("Initializing...");
    display.refresh();
    delay(1000);

    network = new Network(&display);
    network->begin();
    network->startOTATask();

    display.clear();
    display.print("Robot #", 0, 0);
    display.print(network->getRobotNumber());
    display.setCursor(0, 16);
    display.print("Setup...");
    display.refresh();
    delay(1000);

    display.clear();
    display.print("Init AHRS...", 0, 0);
    display.refresh();

    if (!ahrs.begin())
    {
        display.setCursor(0, 16);
        display.print("AHRS Failed!");
        display.refresh();
        while (1);
    }

    display.setCursor(0, 16);
    display.print("AHRS OK");
    display.refresh();
    delay(1000);

    ahrs.calibrate();
    // ahrs.calibrateMag();

   
    display.clear();
    display.print("Init Servos...", 0, 0);
    display.refresh();

    servoControl.begin();
    // servoControl.moveUpSmooth(90);
    // servoControl.moveDownSmooth(90);
    // delay(500);

    display.setCursor(0, 16);
    display.print("Servos OK");
    display.refresh();
    delay(1000);

  
    training.begin();
    training.resetModel();

    display.clear();
    display.print("Training...", 0, 0);
    display.refresh();

   
    training.startTraining();

    display.clear();
    display.print("Training Done", 0, 0);
    display.refresh();
    delay(1500);

    healthCheck.run();

    // ahrs.resetMeasurement();

    display.clear();
    display.print("Run Mode", 0, 0);
    display.refresh();
}

void loop()
{
    ahrs.update();
    display.clear();
    display.print("befoe if",0,0);
    display.refresh();
    if (training.isTraining())
    {
        display.clear();
        display.print("in if",0,0);
        display.refresh();
        return;
    }
    display.clear();
    display.print("outof if",0,0);
    display.refresh();
        

    static unsigned long lastPolicy = 0;
    const unsigned long POLICY_PERIOD_MS = 350;

    unsigned long now = millis();
    if (training.hasLearnedBehavior() && (now - lastPolicy >= POLICY_PERIOD_MS))
    {
        lastPolicy = now;
        training.executeLearnedBehavior();
    }

    delay(5);

    // ahrs.update();
    // if(training.isTraining())
    // {
    //     return;
    // }

    // if(training.hasLearnedBehavior())
    // {
    //     training.executeLearnedBehavior();
    // }
    // delay(70);
}


// ///////////////////////////////////////// Third Version ////////////////////////////////////////





// #include <Arduino.h>

// #include <Display.h>
// #include <AHRS.h>
// #include <ServoControl.h>
// #include <Network.h>
// #include <Training.h>
// #include <HealthCheck.h>

// const uint8_t SERVO_PIN_DOWN = 16;
// const uint8_t SERVO_PIN_UP   = 15;

// Display display;
// AHRS ahrs;
// ServoControl servoControl(SERVO_PIN_DOWN, SERVO_PIN_UP);

// Network* network;

// Training training(&servoControl, &ahrs);
// HealthCheck healthCheck(&display, &ahrs, &servoControl);

// // ====== LCD live IMU debug ======
// void updateIMUDisplay()
// {
//     static unsigned long lastLcd = 0;
//     const unsigned long LCD_PERIOD_MS = 250; // هر 250ms یکبار (فلیکر کمتر)

//     unsigned long now = millis();
//     if (now - lastLcd < LCD_PERIOD_MS) return;
//     lastLcd = now;

//     float ax = ahrs.getAccelX();
//     float ay = ahrs.getAccelY();
//     float az = ahrs.getAccelZ();
//     float amag = ahrs.getAccelMagnitude();

//     float pitch = ahrs.getPitch();
//     float roll  = ahrs.getRoll();
//     // float yaw = ahrs.getYaw(); // اگر خواستی نمایش بدی

//     int upAngle   = servoControl.getCurrentUpAngle();    // بازوی بالایی
//     int downAngle = servoControl.getCurrentDownAngle();  // بازوی پایینی

//     display.clear();

//     // Line 1
//     display.setCursor(0, 0);
//     display.print("Ax:");
//     display.print(ax, 2);
//     display.print(" Ay:");
//     display.print(ay, 2);

//     // Line 2
//     display.setCursor(0, 16);
//     display.print("Az:");
//     display.print(az, 2);
//     display.print(" |A|:");
//     display.print(amag, 2);

//     // Line 3 (orientation)
//     display.setCursor(0, 32);
//     display.print("P:");
//     display.print(pitch, 1);
//     display.print(" R:");
//     display.print(roll, 1);

//     // Line 4 (servo angles)
//     display.setCursor(0, 48);
//     display.print("UP:");
//     display.print(upAngle);
//     display.print(" DN:");
//     display.print(downAngle);

//     display.refresh();
// }

// void setup()
// {
//     Serial.begin(115200);
//     delay(1000);

//     display.begin();
//     display.clear();
//     display.print("RL Robot V2", 0, 0);
//     display.setCursor(0, 16);
//     display.print("Initializing...");
//     display.refresh();
//     delay(1000);

//     network = new Network(&display);
//     network->begin();
//     network->startOTATask();

//     display.clear();
//     display.print("Robot #", 0, 0);
//     display.print(network->getRobotNumber());
//     display.setCursor(0, 16);
//     display.print("Setup...");
//     display.refresh();
//     delay(1000);

//     display.clear();
//     display.print("Init AHRS...", 0, 0);
//     display.refresh();

//     if (!ahrs.begin())
//     {
//         display.setCursor(0, 16);
//         display.print("AHRS Failed!");
//         display.refresh();
//         while (1);
//     }

//     display.setCursor(0, 16);
//     display.print("AHRS OK");
//     display.refresh();
//     delay(1000);

//     ahrs.calibrateAccelGyro();
//     // ahrs.calibrateMag();

//     display.clear();
//     display.print("Init Servos...", 0, 0);
//     display.refresh();

//     servoControl.begin();

//     display.setCursor(0, 16);
//     display.print("Servos OK");
//     display.refresh();
//     delay(1000);

//     training.begin();
//     training.resetModel();

//     display.clear();
//     display.print("Training...", 0, 0);
//     display.refresh();
//     display.clear();
//     display.print("start Training...", 0, 0);
//     display.refresh();
//     delay(200); 
//     training.startTraining();

//     display.clear();
//     display.print("Training Done", 0, 0);
//     display.refresh();
//     delay(1500);

//     healthCheck.run();

//     ahrs.resetMeasurement();

//     display.clear();
//     display.print("Run Mode", 0, 0);
//     display.refresh();
// }

// void loop()
// {
//     ahrs.update();

//     updateIMUDisplay();

//     if (training.isTraining())
//         return;

//     static unsigned long lastPolicy = 0;
//     const unsigned long POLICY_PERIOD_MS = 350;

//     unsigned long now = millis();
//     if (training.hasLearnedBehavior() && (now - lastPolicy >= POLICY_PERIOD_MS))
//     {
//         lastPolicy = now;
//         training.executeLearnedBehavior();
//     }

//     delay(5);
// }
