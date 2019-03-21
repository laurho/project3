// Proto code for PID controller LAB
// Original code by Kenny Breuer, Feb 2017
// Updates by Lauren Ho, March 2019

#define NINT 100            // number of samples to keep for integral control
#define REPORT_TIME 1000    // time btwn serial port reporting time (ms)
#define FLIP_TIME 2000      // time btwn flipping btwn setpoints
#define DELAY_TIME 3      // bandwidth of cntrl loop, controls the delay in loop(), smaller is better

//Note: negative error when less than desired position 0-1023
int SET_A = 15; // first desired position (setpoint A)
int SET_B = 54; // second desired position (setpoint B)
int SET_C = 75; // third desired position (setpoint C)
int SET_D = 90; // fourth desired position (setpoint D)

float p_gain = 0.2; // better performance, but makes overshoot .3, 0.006
float d_gain = 0.5; // control overshoot .1, 0.00001
float i_gain = 0.0; // drift error decrease, stabilize the loop .1, 0.084

float error_history[NINT];
int ierr = 0;             // index of the most current position in the history buff
int iloop = 0;            // count no. of control loops
int setpoint = SET_A;     // init setpoint

unsigned long curr_time;
unsigned long old_time;
unsigned long dT;
unsigned long t_flip;
unsigned long t_report;
float error = 0.0;
float old_error = 0.0;
int n_report = 0;


int mtr_fwd = 3;
int mtr_bwd = 11;
int potentiometer_pin = A0;

//int mtr_fwd = 5;
//int mtr_bwd = 6;
//int potentiometer_pin = A1;

//int mtr_fwd = 9;
//int mtr_bwd = 10;
//int potentiometer_pin = A2;

int output_low = -30;
int output_high = 30;
float motor_stop = 0.75;

void setup() {
  Serial.begin(9600);
  //init the error to 0
  for (int i = 0; i < NINT; i++) {
    error_history[i] = 0;
  }

  //init the time
  curr_time = millis();
  t_flip = curr_time;
  t_report = curr_time;

  //init the motor pins
  pinMode(mtr_fwd, OUTPUT);
  pinMode(mtr_bwd, OUTPUT);
  digitalWrite(mtr_fwd, LOW);
  digitalWrite(mtr_bwd, LOW);
}

void loop() {
  iloop++;  // increase loop counter value

  old_time = curr_time;
  curr_time = millis();
  dT = curr_time - old_time;  //time loop started-time from previous loop

  // cycle though th setpoints based on the flip time
  if (curr_time - t_flip > FLIP_TIME) {
    t_flip = curr_time;
    if (setpoint == SET_A) {
      setpoint = SET_B;
    } else if (setpoint == SET_B) {
      setpoint = SET_C;
    } else if (setpoint == SET_C) {
      setpoint = SET_D;
    } else {
      setpoint = SET_A;
    }
  }

  // update the position and the error
  int pos = analogRead(potentiometer_pin);
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
    int val = map(output, output_low, 0, 80, 250); //May need to change with weight
//    analogWrite(mtr_fwd, val);
//    analogWrite(mtr_bwd, LOW);
  } else if (output > motor_stop) {
    // mtr_bwd
    int val = map(output, 0, output_high, 80, 250); //May need to change with weight
//    analogWrite(mtr_fwd, LOW);
//    analogWrite(mtr_bwd, val);
  } else {
    // output is in a reasonable range, stop the engine
    analogWrite(mtr_fwd, LOW);
    analogWrite(mtr_bwd, LOW);
  }

  delay(DELAY_TIME); //this delay controls the bandwidth of the controller

  // Update to serial port only once in a while since it will cause slow down
  if (curr_time - t_report > REPORT_TIME) {
    Serial.print("Number of loops:"); Serial.print(iloop - n_report);
    Serial.print("Control Bandwidth: "); Serial.print(1000.0 * (iloop - n_report) / (curr_time - t_report));
    Serial.print(" [Hz]; Pos: "); Serial.print(pos);
    Serial.print("; Err: ");      Serial.print(error);
    Serial.print("; dEdT: ");     Serial.print(dEdT);
    Serial.print("; Int: ");      Serial.print(error_int);
    Serial.print("; Out: ");      Serial.println(output);
    t_report = curr_time;
    n_report = iloop;
  }
}
