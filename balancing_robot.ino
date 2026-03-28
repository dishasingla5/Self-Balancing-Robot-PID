// -------------------- STEPPER --------------------
#include <AccelStepper.h>

// -------------------- IMU --------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// -------------------- PID --------------------
#include <PID_v1.h>

// Stepper setup (Driver mode: STEP + DIR)
AccelStepper stepperLeft(2, 11, 12);
AccelStepper stepperRight(2, 9, 10);

// Timing
unsigned long steptimer = 0;
unsigned long oldsteptimer = 0;

// MPU variables
MPU6050 mpu;
#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

// Orientation
float roll;

// -------------------- PID --------------------
double Pk = 200;
double Ik = 0;
double Dk = 1;   

double Setpoint = 0.0, Input, Output;
PID pid(&Input, &Output, &Setpoint, Pk, Ik, Dk, REVERSE);

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(8, INPUT_PULLUP);

  // I2C Setup
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  // MPU Init
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Offsets (your calibrated values)
  mpu.setXGyroOffset(177);
  mpu.setYGyroOffset(-30);
  mpu.setZGyroOffset(-89);
  mpu.setXAccelOffset(-4105);
  mpu.setYAccelOffset(-2437);
  mpu.setZAccelOffset(507);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  // PID setup
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1000, 1000);
  pid.SetSampleTime(10);

  // Stepper setup
  stepperRight.setMaxSpeed(1000);
  stepperLeft.setMaxSpeed(1000);
}

// -------------------- LOOP --------------------
void loop() {

  if (!dmpReady) return;

  // Allow IMU to stabilize at startup
  if (millis() < 3000) return;

  // Get roll angle
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    roll = ypr[2] * 180 / M_PI;
  }

  // Dead zone (reduce jitter)
  if (abs(roll) < 0.5) roll = 0;

  // SAFETY: stop if fallen
  if (abs(roll) > 30) {
    stepperLeft.setSpeed(0);
    stepperRight.setSpeed(0);
    return;
  }

  // PID computation
  Input = roll;
  pid.Compute();

  // Clamp output (extra safety)
  Output = constrain(Output, -800, 800);

  // Ignore very small outputs
  if (abs(Output) < 5) Output = 0;

  // Debug (for Serial Plotter)
  Serial.print(roll);
  Serial.print(",");
  Serial.println(Output);

  // Control motors every 10 ms
  steptimer = millis();
  if ((steptimer - oldsteptimer) > 10) {
    oldsteptimer = steptimer;

    stepperRight.setSpeed(Output * 10);
    stepperLeft.setSpeed(-Output * 10);
  }

  stepperRight.runSpeed();
  stepperLeft.runSpeed();
}