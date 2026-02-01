#include "AHRS.h"

AHRS::AHRS() : initialized(false), isStatic(true), stationaryStartTime(0)
{
    mpu = new MPU9250();
    for (int i = 0; i < 3; i++)
        velocity[i] = position[i] = linearAccel[i] = 0;
}

AHRS::~AHRS()
{
    delete mpu;
}

bool AHRS::begin()
{
    Wire.begin();
    if (!mpu->setup(0x68))
        return false;

    initialized = true;
    lastUpdateTime = millis();
    return true;
}

void AHRS::calibrate()
{
    if (!initialized)
        return;
    mpu->calibrateAccelGyro();
    mpu->calibrateMag();
}

void AHRS::update()
{
    if (!initialized || !mpu->update())
        return;

    unsigned long now = millis();
    float dt = (now - lastUpdateTime) * 0.001f;
    lastUpdateTime = now;

    if (dt <= 0 || dt > 0.1f)
        return;

    // Get raw accelerometer in body frame (units: g, convert to m/s²)
    float AccX = mpu->getAccX() * G_CONST;
    float AccY = mpu->getAccY() * G_CONST;
    float AccZ = mpu->getAccZ() * G_CONST;


    linearAccel[0] = mpu->getLinearAccX();
    linearAccel[1] = mpu->getLinearAccY();
    linearAccel[2] = mpu->getLinearAccZ();

    // Calculate magnitudes for motion detection
    float accNorm = sqrt(
        linearAccel[0] * linearAccel[0] +
        linearAccel[1] * linearAccel[1] +
        linearAccel[2] * linearAccel[2]);
    

    float gyroNorm = sqrt(
        sq(mpu->getGyroX()) +
        sq(mpu->getGyroY()) +
        sq(mpu->getGyroZ()));

    // ZUPT: Zero Velocity Update
    if (accNorm < MOTION_THRESHOLD && gyroNorm < GYRO_THRESHOLD)
    {
        if (stationaryStartTime == 0)
        {
            stationaryStartTime = now;
        }

        // If stationary for sufficient time, zero velocity
        if (now - stationaryStartTime > STATIONARY_TIME_MS)
        {
            isStatic = true;
            velocity[0] = velocity[1] = velocity[2] = 0;
            return;
        }
    }
    else
    {
        stationaryStartTime = 0;
        isStatic = false;
    }

    // 6️⃣ Integration (only when moving)
    if (!isStatic)
    {
        for (int i = 0; i < 3; i++)
        {
            velocity[i] += linearAccel[i] * dt;
            position[i] += velocity[i] * dt;
        }
    }
}

bool AHRS::isMoving()
{
    return !isStatic;
}

void AHRS::resetPosition()
{
    for (int i = 0; i < 3; i++)
        velocity[i] = position[i] = 0;
}


// #include "AHRS.h"
// #include <Arduino.h>
// #include <math.h>

// static constexpr float G_SI = 9.80665f;

// // ---- bias / stationary helpers (فقط داخل همین فایل) ----
// static bool  g_biasReady = false;
// static float g_biasAx = 0.0f, g_biasAy = 0.0f, g_biasAz = 0.0f;   // m/s^2
// static int   g_stillCount = 0;

// static float g_lastLinAx = 0.0f, g_lastLinAy = 0.0f, g_lastLinAz = 0.0f; // برای trapezoid
// static float g_lastLinMag = 0.0f;

// static float accelScaleToSI(float ax, float ay, float az)
// {
//     float mag = sqrtf(ax*ax + ay*ay + az*az);

//     // اگر حدود 1 باشه یعنی g
//     if (mag < 3.0f) return G_SI;

//     // اگر حدود 9.8 باشه یعنی m/s^2
//     return 1.0f;
// }

// AHRS::AHRS() : initialized(false), calibrated(false),
//                velocityX(0), velocityY(0), velocityZ(0),
//                displacementX(0), displacementY(0), displacementZ(0),
//                lastUpdateTime(0),
//                lastSnapshotX(0), lastSnapshotY(0), lastSnapshotZ(0),
//                lastSnapshotTime(0), speedSum(0), accelSum(0), sampleCount(0)
// {
//     mpu = new MPU9250();
// }

// AHRS::~AHRS()
// {
//     delete mpu;
// }

// bool AHRS::begin()
// {
//     Wire.begin();

//     if (!mpu->setup(0x68))
//     {
//         Serial.println("MPU9250 connection failed");
//         initialized = false;
//         return false;
//     }

//     Serial.println("MPU9250 initialized successfully");
//     initialized = true;

//     lastUpdateTime = millis();
//     lastSnapshotTime = millis();

//     // reset bias/stationary trackers
//     g_biasReady = false;
//     g_biasAx = g_biasAy = g_biasAz = 0.0f;
//     g_stillCount = 0;
//     g_lastLinAx = g_lastLinAy = g_lastLinAz = 0.0f;
//     g_lastLinMag = 0.0f;

//     return true;
// }

// void AHRS::update()
// {
//     if (!initialized) return;
//     if (!mpu->update()) return;

//     unsigned long now = millis();
//     float dt = (now - lastUpdateTime) / 1000.0f;
//     lastUpdateTime = now;

//     // اگر dt خیلی بزرگ شد انتگرال خراب نشه
//     if (!(dt > 0.0f && dt < 0.2f)) return;

//     // ----- raw accel -----
//     float ax = getAccelX();
//     float ay = getAccelY();
//     float az = getAccelZ();

//     // convert to SI
//     float scale = accelScaleToSI(ax, ay, az);
//     float axSI = ax * scale;
//     float aySI = ay * scale;
//     float azSI = az * scale;

//     // ----- stationary detection: |a|-g نزدیک صفر + gyro کم -----
//     float amag = sqrtf(axSI*axSI + aySI*aySI + azSI*azSI);
//     float netMag = fabsf(amag - G_SI);

//     float gx = getGyroX();
//     float gy = getGyroY();
//     float gz = getGyroZ();
//     float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);

//     // اینا رو ممکنه کمی تیون کنی
//     const float NET_STILL_TH  = 0.25f;  // m/s^2  (هر چی کوچیک‌تر، سخت‌گیرتر)
//     const float GYRO_STILL_TH = 1.5f;   // اگر gyro deg/s باشه خوبه

//     bool stationary = (netMag < NET_STILL_TH) && (gyroMag < GYRO_STILL_TH);

//     if (stationary) g_stillCount++;
//     else g_stillCount = 0;

//     // ----- bias learn (فقط وقتی ساکنه) -----
//     // bias همان بردار (gravity + bias) در همان وضعیت نصب سنسور است.
//     if (stationary)
//     {
//         const float beta = 0.02f; // سرعت یادگیری bias
//         if (!g_biasReady)
//         {
//             // چند فریم اول bias را سریع بگیر
//             g_biasAx = axSI;
//             g_biasAy = aySI;
//             g_biasAz = azSI;
//             if (g_stillCount > 25) g_biasReady = true;
//         }
//         else
//         {
//             g_biasAx = (1.0f - beta) * g_biasAx + beta * axSI;
//             g_biasAy = (1.0f - beta) * g_biasAy + beta * aySI;
//             g_biasAz = (1.0f - beta) * g_biasAz + beta * azSI;
//         }
//     }

//     // اگر bias هنوز آماده نیست، فعلاً انتگرال نگیر (وگرنه drift می‌گیری)
//     if (!g_biasReady)
//     {
//         // برای averaging
//         speedSum += getSpeed();
//         accelSum += netMag;
//         sampleCount++;
//         return;
//     }

//     // ----- linear accel = raw - bias -----
//     float linAx = axSI - g_biasAx;
//     float linAy = aySI - g_biasAy;
//     float linAz = azSI - g_biasAz;

//     // deadband (نویز زیر این مقدار را صفر کن)
//     const float DEAD = 0.12f; // m/s^2
//     if (fabsf(linAx) < DEAD) linAx = 0;
//     if (fabsf(linAy) < DEAD) linAy = 0;
//     if (fabsf(linAz) < DEAD) linAz = 0;

//     float linMag = sqrtf(linAx*linAx + linAy*linAy + linAz*linAz);
//     g_lastLinMag = linMag;

//     // ----- ZUPT: اگر چند فریم پشت سر هم ساکن بود، سرعت را صفر کن -----
//     if (g_stillCount > 15)
//     {
//         velocityX = velocityY = velocityZ = 0.0f;

//         // جلوگیری از جهش بعد از سکون
//         g_lastLinAx = linAx;
//         g_lastLinAy = linAy;
//         g_lastLinAz = linAz;

//         // averaging
//         speedSum += 0.0f;
//         accelSum += linMag;
//         sampleCount++;
//         return;
//     }

//     // ----- integrate (trapezoid) -----
//     velocityX += 0.5f * (linAx + g_lastLinAx) * dt;
//     velocityY += 0.5f * (linAy + g_lastLinAy) * dt;
//     velocityZ += 0.5f * (linAz + g_lastLinAz) * dt;

//     // damping کوچک برای drift کمتر
//     const float V_DAMP = 0.995f;
//     velocityX *= V_DAMP;
//     velocityY *= V_DAMP;
//     velocityZ *= V_DAMP;

//     displacementX += velocityX * dt;
//     displacementY += velocityY * dt;
//     displacementZ += velocityZ * dt;

//     g_lastLinAx = linAx;
//     g_lastLinAy = linAy;
//     g_lastLinAz = linAz;

//     // averaging
//     speedSum += getSpeed();
//     accelSum += linMag;
//     sampleCount++;
// }

// bool AHRS::isMoving()
// {
//     if (!initialized) return false;

//     // MOTION_THRESHOLD باید بر حسب m/s^2 باشد
//     // اگر در AHRS.h مقدارش خیلی کم/زیاد است، اینجا رفتار عجیب می‌شود
//     return g_lastLinMag > MOTION_THRESHOLD;
// }

// float AHRS::getDisplacementX() { return displacementX; }
// float AHRS::getDisplacementY() { return displacementY; }
// float AHRS::getDisplacementZ() { return displacementZ; }

// void AHRS::resetDisplacement()
// {
//     velocityX = velocityY = velocityZ = 0;
//     displacementX = displacementY = displacementZ = 0;
//     lastUpdateTime = millis();
// }

// float AHRS::getVelocityX() { return velocityX; }
// float AHRS::getVelocityY() { return velocityY; }
// float AHRS::getVelocityZ() { return velocityZ; }

// float AHRS::getSpeed()
// {
//     return sqrtf(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);
// }

// float AHRS::getAccelMagnitude()
// {
//     // شتاب خطی (بدون گرانش) برای debug/Reward بهتره
//     return g_lastLinMag;
// }

// float AHRS::getRoll()  { return initialized ? mpu->getRoll()  : 0; }
// float AHRS::getPitch() { return initialized ? mpu->getPitch() : 0; }
// float AHRS::getYaw()   { return initialized ? mpu->getYaw()   : 0; }

// float AHRS::getAccelX() { return initialized ? mpu->getAccX() : 0; }
// float AHRS::getAccelY() { return initialized ? mpu->getAccY() : 0; }
// float AHRS::getAccelZ() { return initialized ? mpu->getAccZ() : 0; }

// float AHRS::getGyroX() { return initialized ? mpu->getGyroX() : 0; }
// float AHRS::getGyroY() { return initialized ? mpu->getGyroY() : 0; }
// float AHRS::getGyroZ() { return initialized ? mpu->getGyroZ() : 0; }

// float AHRS::getMagX() { return initialized ? mpu->getMagX() : 0; }
// float AHRS::getMagY() { return initialized ? mpu->getMagY() : 0; }
// float AHRS::getMagZ() { return initialized ? mpu->getMagZ() : 0; }

// float AHRS::getQuatW() { return initialized ? mpu->getQuaternionW() : 1; }
// float AHRS::getQuatX() { return initialized ? mpu->getQuaternionX() : 0; }
// float AHRS::getQuatY() { return initialized ? mpu->getQuaternionY() : 0; }
// float AHRS::getQuatZ() { return initialized ? mpu->getQuaternionZ() : 0; }

// float AHRS::getTemperature() { return initialized ? mpu->getTemperature() : 0; }

// void AHRS::calibrateAccelGyro()
// {
//     if (!initialized) return;
//     Serial.println("Calibrating Accel & Gyro... Keep the sensor still");
//     mpu->calibrateAccelGyro();
//     Serial.println("Accel & Gyro calibration complete");
// }

// void AHRS::calibrateMag()
// {
//     if (!initialized) return;
//     Serial.println("Calibrating Magnetometer... Wave the sensor in a figure 8");
//     mpu->calibrateMag();
//     Serial.println("Magnetometer calibration complete");
//     calibrated = true;
// }

// bool AHRS::isCalibrated() { return calibrated; }

// AHRS::MovementSnapshot AHRS::getMeasurement()
// {
//     MovementSnapshot snapshot;

//     unsigned long currentTime = millis();
//     snapshot.deltaTime = (currentTime - lastSnapshotTime) / 1000.0f;

//     float dx = displacementX - lastSnapshotX;
//     float dy = displacementY - lastSnapshotY;
//     float dz = displacementZ - lastSnapshotZ;

//     snapshot.deltaDistance = sqrtf(dx * dx + dy * dy + dz * dz) * 100.0f; // m -> cm

//     if (sampleCount > 0)
//     {
//         snapshot.avgSpeed = (speedSum / sampleCount) * 100.0f; // m/s -> cm/s
//         snapshot.avgAcceleration = accelSum / sampleCount;     // m/s^2 (linear)
//     }
//     else
//     {
//         snapshot.avgSpeed = 0;
//         snapshot.avgAcceleration = 0;
//     }

//     return snapshot;
// }

// void AHRS::resetMeasurement()
// {
//     lastSnapshotX = displacementX;
//     lastSnapshotY = displacementY;
//     lastSnapshotZ = displacementZ;
//     lastSnapshotTime = millis();

//     speedSum = 0;
//     accelSum = 0;
//     sampleCount = 0;
// }
