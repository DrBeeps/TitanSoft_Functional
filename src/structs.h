#include <Arduino.h>

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
  unsigned long time;

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

  double tvcZ;
  double tvcY;

  double deviationZ;
  double deviationY;

  double altitude;
  
  double battVoltage;


  int DATA_ERROR;
};

