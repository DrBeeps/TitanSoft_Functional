#include <Arduino.h>

#define LOGIC_REF 3.3

#define DIV_HIGH 1000000
#define DIV_LOW 100000
#define DIV_MULT ((LOGIC_REF / 1023) * (DIV_HIGH + DIV_LOW / DIV_LOW));

#define EARTHG 9.807

enum FlightMode
{
  GROUND_IDLE = 1,
  POWERED_FLIGHT = 2,
  UNPOWERED_FLIGHT = 3,
  BALLISTIC_DESCENT = 4,
  CHUTE_DESCENT = 5,
  GROUND_SAFE = 6
};

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
