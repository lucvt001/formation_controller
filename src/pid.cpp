#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "formation_controller/pid.h"

using namespace std;

float PID::calculate( float setpoint, float current )
{
    
  // Calculate error
  float error = setpoint - current;

  // Proportional term
  float Pout = Kp_ * error;

  // Integral term
  if (error * integral_ < -1e-8)  // If the error and integral have different signs
  {
    if (abs(unwinding_factor_ - (-1)) < 1e-5)  // If the unwinding factor is -1, the integral term is instantly reset
      integral_ = 0;
    else  // If the unwinding factor is not -1, multiply the error by the unwinding factor before adding it to the integral (faster anti-windup)
      integral_ += error * dt_ * unwinding_factor_;
  } else  // If the error and integral have the same sign, add the error to the integral as normal
    integral_ += error * dt_;

  float Iout = Ki_ * integral_;

  // Derivative term
  float derivative = (error - prev_error_) / dt_;
  float Dout = Kd_ * derivative;

  // Calculate total output
  float output = Pout + Iout + Dout;

  // Restrict to max/min
  if( output > max_ )
    output = max_;
  else if( output < min_ )
    output = min_;

  // Save error to previous error
  prev_error_ = error;

  if ( verbose_mode_ )
    // cout << pid_name_ << " - error: " << error << " integral: " << integral_ << " derivative: " << derivative << " Output: " << output << endl;
    cout << pid_name_ << " - Pout: " << Pout << " Iout: " << Iout << " Dout: " << Dout << " Output: " << output << endl;

  return output;
}

void PID::set_unwinding_factor( float unwinding_factor )
{
  unwinding_factor_ = unwinding_factor;
}

void PID::reset()
{
  prev_error_ = 0.0;
  integral_ = 0.0;
}

void PID::enable_verbose_mode(bool is_enable, string pid_name)
{
  verbose_mode_ = is_enable;
  pid_name_ = pid_name;
}

void PID::set_gains( float Kp, float Ki, float Kd )
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

#endif