#include "Arduino.h"
#include "ServoMotor.h"

ServoMotor::ServoMotor(int fwdPin, int bwdPin, int potPin) {
  mtr_fwd = fwdPin;
  mtr_bwd = bwdPin;
  pot_pin = potPin;
}

void ServoMotor::setSetpoint(int setA) {
  setpoint = setA;
}

void ServoMotor::setPID(float p, float i, float d) {
  p_gain = p;
  i_gain = i;
  d_gain = d;
}

void ServoMotor::setupMotor() {
  //init the error to 0
  for (int i = 0; i < NINT; i++) {
    error_history[i] = 0;
  }

  //init the time
  curr_time = millis();
  t_flip = curr_time;
  t_report = curr_time;

  //init other values
  ierr = 0;
  iloop = 0;
  error = 0.0;
  old_error = 0.0;
  n_report = 0;
  output_low = -30;
  output_high = 30;
  motor_stop = 0.75;
  motor_high = 250;
  motor_low = 80;
  finished = false;

  //init the motor pins
  pinMode(mtr_fwd, OUTPUT);
  pinMode(mtr_bwd, OUTPUT);
  pinMode(pot_pin, INPUT);
  digitalWrite(mtr_fwd, LOW);
  digitalWrite(mtr_bwd, LOW);
}

void ServoMotor::moveToSetpoint() {
  if (true) {
    iloop++;  // increase loop counter value

    old_time = curr_time;
    curr_time = millis();
    dT = curr_time - old_time;  //time loop started-time from previous loop

    // update the position and the error
    int pos = analogRead(pot_pin);
    pos = pos / 10; //take pos/10 to decrease the mapping error
    old_error = error;
    error = pos - setpoint;
    error_history[ierr] = error;
    ierr++;
    ierr = ierr % NINT;
    // update the error integral
    float error_int = 0;
    for (int i = 0; i < NINT; i++) {
      error_int += error_history[i];
    }
    error_int *= (float) dT / NINT;
    float dEdT = (float) (error - old_error) / dT;

    float output = p_gain * error + d_gain * dEdT + i_gain * error_int;
    output = constrain(output, output_low, output_high);
    if (output < -motor_stop) {
      // mtr_fwd
      int val = map(output, output_low, 0, motor_low, motor_high); //May need to change with weight
      analogWrite(mtr_fwd, val);
      analogWrite(mtr_bwd, LOW);
    } else if (output > motor_stop) {
      // mtr_bwd
      int val = map(output, 0, output_high, motor_low, motor_high); //May need to change with weight
      analogWrite(mtr_fwd, LOW);
      analogWrite(mtr_bwd, val);
    } else {
      // output is in a reasonable range, stop the engine
      analogWrite(mtr_fwd, LOW);
      analogWrite(mtr_bwd, LOW);
      finished = true;
    }

    //delay(DELAY_TIME); //this delay controls the bandwidth of the controller

    // Update to serial port only once in a while since it will cause slow down
    if (curr_time - t_report > REPORT_TIME) {
      Serial.print("Number of loops:"); Serial.print(iloop - n_report);
      Serial.print(" Control Bandwidth: "); Serial.print(1000.0 * (iloop - n_report) / (curr_time - t_report));
      Serial.print(" [Hz]; Pos: "); Serial.print(pos);
      Serial.print("; Err: ");      Serial.print(error);
      Serial.print("; dEdT: ");     Serial.print(dEdT);
      Serial.print("; Int: ");      Serial.print(error_int);
      Serial.print("; Out: ");      Serial.println(output);
      t_report = curr_time;
      n_report = iloop;
    }
  }

}
