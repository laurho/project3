// Proto code for PID controller LAB
// Original code by Kenny Breuer, Feb 2017
// Updates by Lauren Ho and Bessie Jiang, March 2019
#ifndef ServoMotor_h
#define ServoMotor_h

#include "Arduino.h"

#define NINT 100            // number of samples to keep for integral control
#define REPORT_TIME 1000    // time btwn serial port reporting time (ms)
#define FLIP_TIME 2000      // time btwn flipping btwn setpoints
#define DELAY_TIME 3      // bandwidth of cntrl loop, controls the delay in loop(), smaller is better

class ServoMotor {
private:
  float p_gain;
  float d_gain;
  float i_gain;
  
  float error_history[NINT];
  int ierr;
  int iloop;
  int setpoint;
  
  unsigned long curr_time;
  unsigned long old_time;
  unsigned long dT;
  unsigned long t_flip;
  unsigned long t_report;
  
  float error;
  float old_error;
  int n_report;
  
  int SET_A;
  int SET_B;
  
  int mtr_fwd;
  int mtr_bwd;
  int pot_pin;
  
  int output_low;
  int output_high;
  
  float motor_stop;
  int motor_high;
  int motor_low;

public:
  ServoMotor(int fwdPin, int bwdPin, int potPin);
  void setupMotor();
  void setPID(float p, float i, float d);
  void setSetpoints(int setA, int setB);
  void loopMotor();
};

#endif