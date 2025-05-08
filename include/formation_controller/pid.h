#ifndef _PID_H_
#define _PID_H_

using namespace std;

class PID
{
public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  // PID( float Kp, float Ki, float Kd, float dt, float max, float min );
  PID ( float Kp, float Ki, float Kd, float dt, float max, float min, float scale = 1.0 )
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt), max_(max), min_(min), scale_(scale),
    prev_error_(0.0), integral_(0.0),
    unwinding_factor_(1.0),
    pid_name_("PID"), verbose_mode_(false) {};

  // Returns the manipulated variable given a setpoint and current process value
  float calculate( float setpoint, float current );

  // Set the unwinding factor for the integral term
  void set_unwinding_factor( float unwinding_factor );

  // Enable verbose mode to print Pout, Iout, Dout and output
  void enable_verbose_mode( bool is_enable, string pid_name );

  // Set the PID gains
  void set_gains( float Kp, float Ki, float Kd );

  // Reset the integrator and the previous error
  void reset();

private:
  float Kp_;
  float Ki_;
  float Kd_;
  float dt_;
  float max_;
  float min_;
  float scale_;
  float prev_error_;
  float integral_;
  float unwinding_factor_;

  string pid_name_;
  bool verbose_mode_;
};

#endif