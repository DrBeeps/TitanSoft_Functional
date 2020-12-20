#include <Arduino.h>
#include <Servo.h>
#include <Orientation.h>
#include <pid.h>
#include <Wire.h> 
#include <SD.h>
#include <SPI.h>
#include <BMI088.h>
#include "Orientation.h"

#include "structs.h"


// IMU Hardware //
Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);
// ============ //

const int chipSelect = BUILTIN_SDCARD;

// Servo Hardware //
Servo servoZ;
Servo servoY;
// ============== //

// Euler Angles & Quaternions //
Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;
// ========================== //

// Time //
uint64_t lastMicros;
uint64_t currentMicros;
double dt;
// ==== //

double IMUValX = 0, IMUValY = 0, IMUValZ = 0; // raw offset values
double IMUOffsetX = 0, IMUOffsetY = 0, IMUOffsetZ = 0; // final offset values
int IMUSampleTime = 0;
double LocalOrientationX = 0;
double LocalOrientationY = 0;
double LocalOrientationZ = 0;

double accelX, accelY, accelZ;
double gyroX, gyroY, gyroZ;

double flightAlt;
double fcBatt;
bool pyro1Cont;
const int pyro4 = 33;
bool pyro2Cont;

int status;

double kp = 0.35;
double ki = 0.095;
double kd = 0.174;
// gains tested by the flight simulation

PID zAxis = {kp, ki, kd, 0};
PID yAxis = {kp, ki, kd, 0};  
double pwmZ, pwmY;
double trueYOut, trueZOut;
double SGR = 6; // the BPS Mount SGR is very high

FlightMode currentMode = GROUND_IDLE;

String timeString;
String flightModeString;
String gXString;
String gYString;
String gZString;
String aXString;
String aYString;
String aZString;
String yawString;
String pitchString;
String rollString;
String altString;
String battString;
String py1String;
String py2String;

uint32_t flightDurationTime;



File sdFlightData;

void servoHome()
{
  servoZ.write(90);
  servoY.write(90);
}


void setupSD()
{
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}


void stabilize(double dt)
{
  gyro.readSensor();

  gyroData.roll = (gyro.getGyroX_rads());
  gyroData.pitch = (gyro.getGyroY_rads());
  gyroData.yaw = (gyro.getGyroZ_rads());

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);

  // Serial.print("ORE Z => "); Serial.print(LocalOrientationZ); Serial.print("\t");
  // Serial.print("ORE Y => "); Serial.print(LocalOrientationY); Serial.print("\n");

  pwmZ = zAxis.update(LocalOrientationZ, dt);
  pwmY = yAxis.update(LocalOrientationY, dt);

  trueZOut = constrain((int)(pwmZ * RAD_TO_DEG * SGR), -30, 30);
  trueYOut = constrain((int)(pwmY * RAD_TO_DEG * SGR), -30, 30);

  servoZ.write(90 + trueZOut);
  servoY.write(90 + trueYOut);

  // Serial.print("Z OUT"); Serial.print(90 + trueZOut); Serial.print("\t");
  // Serial.print("Y OUT"); Serial.print(90 + trueYOut); Serial.print("\n");
}

bool initAccel()
{
  if(accel.begin() < 0 || accel.setRange(Bmi088Accel::RANGE_12G) < 0 || accel.setOdr(Bmi088Accel::ODR_400HZ_BW_145HZ) < 0) return false;
}

bool initGyro()
{
  if(gyro.begin() < 0 || gyro.setRange(Bmi088Gyro::RANGE_1000DPS) < 0 || gyro.setOdr(Bmi088Gyro::ODR_2000HZ_BW_230HZ) < 0) return false;
}

void setup() 
{
  Serial.begin(9600);

  pinMode(pyro4, OUTPUT);

  servoZ.attach(37);
  servoY.attach(36);

  setupSD();

  while(!Serial) {}

  status = accel.begin();

  if (status < 0) 
  {
      Serial.println("Accel Initialization Error");
      Serial.println(status);
      while (1) {}
  }
  status = gyro.begin();
  if (status < 0) 
  {
      Serial.println("Gyro Initialization Error");
      Serial.println(status);
      while (1) {}
  }
  while(!gyro.getDrdyStatus()) {}

  servoHome();
  delay(1000);

  currentMode = GROUND_IDLE;
  // flightDurationTime = millis();
  lastMicros = micros();
}

void loop()
{
  currentMicros = micros();
  dt = ((double)(currentMicros - lastMicros) / 1000000.);
  stabilize(dt);
  Serial.print("ORE Z"); Serial.print(LocalOrientationZ); Serial.print("\t");
  Serial.print("ORE Y"); Serial.print(LocalOrientationY); Serial.print("\n");
  lastMicros = currentMicros;
}
