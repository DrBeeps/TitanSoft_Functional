#include <Arduino.h>
#include <Servo.h>
#include <Orientation.h>
#include <pid.h>
#include <Wire.h> 
#include <SPI.h>
#include <BMI088.h>
#include "Orientation.h"

#include "SD.h"
#include "structs.h"

// ==============================
// SDCARD & DATA LOG
// ==============================

const int chipSelect = BUILTIN_SDCARD;

const int spiFlashChipSelect = 1;

File sdDataLog;

double flightAlt;
double fcBatt;

int DATA_ERROR;

// ==============================
// SERVO | PYROS | LED
// ==============================

Servo servoZ;
Servo servoY;

Servo esc1;

double SGR = 6; // the BPS Mount SGR is very high

const int pyro1 = 2;
const int pyro2 = 10;
const int pyro3 = 29;
const int pyro4 = 33;

bool pyro1Cont;
bool pyro2Cont;

const int LEDR = 19;
const int LEDG = 16;
const int LEDB = 15;

// ==============================
// GYROSCOPES & ACCELEROMETERS
// ==============================

Bmi088Accel accel(Wire,0x19);
Bmi088Gyro gyro(Wire,0x69);

int status;

// ==============================
// GNC
// ==============================

uint64_t lastMicros;
uint64_t thisLoopMicros;
double dt;

uint32_t nextServoMicros = 0;
const uint8_t servoHz = 16; // This can be changed
const uint32_t servoMicros = 1000000 / servoHz;

Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

double IMUValX = 0, IMUValY = 0, IMUValZ = 0; // raw offset values
double IMUOffsetX = 0, IMUOffsetY = 0, IMUOffsetZ = 0; // final offset values
int IMUSampleTime = 0;
double LocalOrientationX = 0;
double LocalOrientationY = 0;
double LocalOrientationZ = 0;

double accelX, accelY, accelZ;
double gyroX, gyroY, gyroZ;

double kp = 0.55;
double ki = 0.095;
double kd = 0.175; // D gain = weird but I was advised to bring it down from .174 to .074

double setpoint = 0;

double deviationZ;
double deviationY;

PID zAxis = {kp, ki, kd, setpoint};
PID yAxis = {kp, ki, kd, setpoint};  
double pwmZ, pwmY;
double trueYOut, trueZOut;

FlightMode currentMode = GROUND_IDLE;

uint32_t flightDurationTime;



File sdFlightData;

// ==============================
// UNTESTED FUNCTIONS
// ==============================

// ==============================
// 
// ==============================

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

  deviationZ = LocalOrientationZ - setpoint;
  deviationY = LocalOrientationY - setpoint;

  pwmZ = zAxis.update(LocalOrientationZ, dt);
  pwmY = yAxis.update(LocalOrientationY, dt);

  trueZOut = constrain((pwmZ * SGR), -30, 30);
  trueYOut = constrain((pwmY * SGR), -30, 30);

  if(thisLoopMicros >= nextServoMicros)
  {
    servoZ.write(90 + trueZOut);
    servoY.write(90 + trueYOut);

    nextServoMicros += servoMicros;
  }

  ///Serial.print("Z OUT"); Serial.print(90 + trueZOut); Serial.print("\t");
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

void setupIMU () 
{
  if (initAccel() < 0) 
  {
    Serial.println("Accel Initialization Error");
    while (1) {}
  }
  if (initGyro() < 0) 
  {
    Serial.println("Gyro Initialization Error");
    while (1) {}
  }
  while(!gyro.getDrdyStatus()) {}
}

void logHeaders(File dataLoggingFile)
{
  dataLoggingFile = SD.open("STATICFIRE.csv", FILE_WRITE);
  if (dataLoggingFile) 
  {
    dataLoggingFile.print("Flight Time (Millis)"); dataLoggingFile.print(", "); dataLoggingFile.print("System State"); dataLoggingFile.print(", "); dataLoggingFile.print("Gyro X (rad/s)"); dataLoggingFile.print(", "); dataLoggingFile.print("Gyro Y (rad/s)"); dataLoggingFile.print(", "); dataLoggingFile.print("Gyro Z (rad/s)"); dataLoggingFile.print(", "); dataLoggingFile.print("Accel X (m/s^2)"); dataLoggingFile.print(", "); dataLoggingFile.print("Accel Y (m/s^2)"); dataLoggingFile.print(", "); dataLoggingFile.print("Accel Z (m/s^2)"); dataLoggingFile.print(", "); dataLoggingFile.print("Yaw (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("Pitch (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("Roll (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("TVC Z (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("TVC Y (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("Deviation Z (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("Deviation Y (Deg)"); dataLoggingFile.print(", "); dataLoggingFile.print("Altitude (Meters)"); dataLoggingFile.print(", "); dataLoggingFile.print("FC Batt");  dataLoggingFile.print(", "); dataLoggingFile.print("Data Error"); dataLoggingFile.println("");

    dataLoggingFile.close();
  } else 
  {
    Serial.println("error opening STATICFIRE.csv");
  }
}

void logData(FlightData currentData)
{
  sdDataLog = SD.open("STATICFIRE.csv", FILE_WRITE);
  
  // if the file opened okay, write to it:
  String flightDurationString = String(currentData.time);
  String systemStateString = String(currentData.state);
  String gXString = String(currentData.gX);
  String gYString = String(currentData.gY);
  String gZString = String(currentData.gZ);
  String aXString = String(currentData.aX);
  String aYString = String(currentData.aY);
  String aZString = String(currentData.aZ);
  String yawString = String(currentData.yaw);
  String pitchString = String(currentData.pitch);
  String rollString = String(currentData.roll);
  String tvcZString = String(trueZOut);
  String tvcYString = String(trueYOut);
  String deviationZString = String(deviationZ);
  String deviationYString = String(deviationY);
  String altString = String(currentData.altitude);
  String fcBattString = String(currentData.battVoltage);
  String dataErrorString = String(currentData.DATA_ERROR);

  if (sdDataLog) 
  {
    sdDataLog.print(flightDurationString); sdDataLog.print(", "); sdDataLog.print(systemStateString); sdDataLog.print(", "); sdDataLog.print(gXString); sdDataLog.print(", "); sdDataLog.print(gYString); sdDataLog.print(", "); sdDataLog.print(gZString); sdDataLog.print(", "); sdDataLog.print(aXString); sdDataLog.print(", "); sdDataLog.print(aYString); sdDataLog.print(", "); sdDataLog.print(aZString); sdDataLog.print(", "); sdDataLog.print(yawString); sdDataLog.print(", "); sdDataLog.print(pitchString); sdDataLog.print(", "); sdDataLog.print(rollString); sdDataLog.print(", "); sdDataLog.print(tvcZString); sdDataLog.print(", "); sdDataLog.print(tvcYString); sdDataLog.print(", "); sdDataLog.print(deviationZ); sdDataLog.print(", "); sdDataLog.print(deviationY); sdDataLog.print(", "); sdDataLog.print(altString); sdDataLog.print(", "); sdDataLog.print(fcBattString); sdDataLog.print(", "); sdDataLog.print(dataErrorString); sdDataLog.println("");

    sdDataLog.close();
  } else 
  {
    Serial.println("error opening STATICFIRE.csv");
  }
}

void setup() 
{
  Serial.begin(9600);

  pinMode(pyro1, OUTPUT);
  pinMode(pyro2, OUTPUT);
  pinMode(pyro3, OUTPUT);
  pinMode(pyro4, OUTPUT);

  pinMode(A10, INPUT);

  servoZ.attach(37);
  servoY.attach(36);

  delay(1000);

  setupSD();
  logHeaders(sdDataLog);
  
  // esc1.attach(5);
  // esc1.writeMicroseconds(1000);

  servoHome();

  delay(1000);

  currentMode = GROUND_IDLE;

  delay(1000);

  setupIMU();

  // digitalWrite(pyro4, HIGH); // DO NOT UNCOMMENT UNTIL ACTUAL STATIC FIRE CODE UPLOAD!!!
  currentMode = POWERED_FLIGHT;
  nextServoMicros = micros() + servoMicros;
  lastMicros = micros();
}


void loop()
{
  thisLoopMicros = micros();
  dt = ((double)(thisLoopMicros - lastMicros) / 1000000.);

  stabilize(dt);

  accel.readSensor();

  fcBatt = analogRead(A13) * 0.06025510204;

  FlightData currentData = {thisLoopMicros / 1000000., currentMode, gyro.getGyroX_rads(), -gyro.getGyroY_rads(), -gyro.getGyroZ_rads(), accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss(), LocalOrientationX, LocalOrientationY, LocalOrientationZ, pwmZ, pwmY, deviationZ, deviationY, flightAlt, fcBatt, DATA_ERROR};
 
  logData(currentData);

  lastMicros = thisLoopMicros;
} 




