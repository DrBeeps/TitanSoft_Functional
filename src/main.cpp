#include <Arduino.h>
#include <Servo.h>
#include <Orientation.h>
#include <pid.h>
#include <Wire.h> 
#include <SD.h>
#include <SPI.h>
#include <BMI088.h>
#include "Orientation.h"

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
bool pyro2Cont;

int status;

double kp = 0.55;
double ki = 0.25;
double kd = 0.275;
// very aggresive gains tested by the flight simulation

PID zAxis = {kp, ki, kd, 0};
PID yAxis = {kp, ki, kd, 0};  
double pwmZ, pwmY;
double trueYOut, trueZOut;
double SGR = 5; // the BPS Mount SGR is very high
double cs, sn;

enum FlightMode
{
  GROUND_IDLE = 1,
  POWERED_FLIGHT = 2,
  UNPOWERED_FLIGHT = 3,
  BALLISTIC_DESCENT = 4,
  CHUTE_DESCENT = 5,
  GROUND_SAFE = 6
};

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

struct FlightData
{
  uint32_t time;

  FlightMode state;

  double gX;
  double gY;
  double gZ;

  double aX;
  double aY;
  double aZ;

  double yaw;
  double pitch;
  double roll;

  double altitude;
  
  double battVoltage;

  bool pyro1Cont;
  bool pyro2Cont;

  int DATA_ERROR;
};

struct TVCData
{
  int TVCAngleWriteZ;
  int TVCAngleWriteY;


};

File sdFlightData;

void servoHome()
{
  servoZ.write(90);
  servoY.write(90);
}

void setupIMU()
{
  Serial.begin(115200);

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

void writeHeaders(File writeFile)
{
  writeFile.print("Time (Millis)"); Serial.print(",");
  writeFile.print("System State"); Serial.print(",");
  writeFile.print("Gyro X (rad/s)"); Serial.print(",");
  writeFile.print("Gyro Y (rad/s)"); Serial.print(",");
  writeFile.print("Gyro Z (rad/s)"); Serial.print(",");
  writeFile.print("Accel X (m/s^2)"); Serial.print(",");
  writeFile.print("Accel Y (m/s^2)"); Serial.print(",");
  writeFile.print("Accel Z (m/s^2)"); Serial.print(",");
  writeFile.print("Yaw (Deg)"); Serial.print(",");
  writeFile.print("Pitch (Deg)"); Serial.print(",");
  writeFile.print("Roll (Deg)"); Serial.print(",");
  writeFile.print("Altitude (Meters)"); Serial.print(",");
  writeFile.print("FC Batt "); Serial.print(",");
  writeFile.print("Pyro 1 Cont"); Serial.print(",");
  writeFile.println("Pyro 2 Cont"); 
  writeFile.close();
  Serial.println("Wrote headers...");
} 

void logData(FlightData sdDataLog)
{
  timeString = String(sdDataLog.time);
  flightModeString = String(sdDataLog.state);
  gXString = String(sdDataLog.gX);
  gYString = String(sdDataLog.gY);
  gZString = String(sdDataLog.gZ);
  aXString = String(sdDataLog.aX);
  aYString = String(sdDataLog.aY);
  aZString = String(sdDataLog.aZ);
  yawString = String(sdDataLog.yaw);
  pitchString = String(sdDataLog.pitch);
  rollString = String(sdDataLog.roll);
  altString = String(sdDataLog.altitude);
  battString = String(sdDataLog.battVoltage);
  py1String = String(sdDataLog.pyro1Cont);
  py2String = String(sdDataLog.pyro2Cont);

  sdFlightData.print(timeString);
  sdFlightData.print(",");
  sdFlightData.print(flightModeString);
  sdFlightData.print(",");
  sdFlightData.print(gXString);
  sdFlightData.print(",");
  sdFlightData.print(gYString);
  sdFlightData.print(",");
  sdFlightData.print(gZString);
  sdFlightData.print(",");
  sdFlightData.print(aXString);
  sdFlightData.print(",");
  sdFlightData.print(aYString);
  sdFlightData.print(",");
  sdFlightData.print(aZString);
  sdFlightData.print(",");
  sdFlightData.print(yawString);
  sdFlightData.print(",");
  sdFlightData.print(pitchString);
  sdFlightData.print(",");
  sdFlightData.print(rollString);
  sdFlightData.print(",");
  sdFlightData.print(altString);
  sdFlightData.print(",");
  sdFlightData.print(battString);
  sdFlightData.print(",");
  sdFlightData.print(py1String);
  sdFlightData.print(",");
  sdFlightData.println(py2String);
  Serial.println("Wrote data...");
}

void logTestData()
{

}

void testAccel()
{
  accel.readSensor();

  accelX = accel.getAccelX_mss();
  accelY = accel.getAccelY_mss();
  accelZ = accel.getAccelZ_mss();

  Serial.print("Accel X: "); Serial.print(accelX); Serial.print("\t");
  Serial.print("Accel Y: "); Serial.print(accelY); Serial.print("\t");
  Serial.print("Accel Z: "); Serial.print(accelZ); Serial.print("\n");
}

void testGyro()
{
  gyro.readSensor();

  gyroData.roll = (gyro.getGyroX_rads() - abs(IMUOffsetX));
  gyroData.pitch = (-gyro.getGyroZ_rads() - abs(IMUOffsetZ));
  gyroData.yaw = (-gyro.getGyroY_rads() - abs(IMUOffsetY));

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();
  
  LocalOrientationX = (gyroOut.roll * RAD_TO_DEG);
  LocalOrientationY = (gyroOut.pitch * RAD_TO_DEG);
  LocalOrientationZ = (gyroOut.yaw * RAD_TO_DEG);

  Serial.print("LocalOrientationY: "); Serial.print(LocalOrientationY); Serial.print("\t");
  Serial.print("LocalOrientationZ: "); Serial.print(LocalOrientationZ); Serial.print("\n");
}

void stabilize(double dt)
{
  gyro.readSensor();

  gyroData.roll = (gyro.getGyroX_rads());
  gyroData.pitch = (-gyro.getGyroY_rads());
  gyroData.yaw = (-gyro.getGyroZ_rads());

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

void setup() 
{
  Serial.begin(9600);
  servoZ.attach(37);
  servoY.attach(36);

  setupSD();

  setupIMU();
  delay(1000);
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
