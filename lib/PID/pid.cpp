#include "pid.h"

double PID::update(double input, double dt)
{
    double error = setpoint - input;
    integral += error * dt;

    double derivative = (error - prevError) / dt;
    prevError = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}
 