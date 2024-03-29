
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define NINT 100            // number of samples to keep for integral control
#define REPORT_TIME 3000    // time btwn serial port reporting time (ms)
#define FLIP_TIME 2000      // time btwn flipping btwn setpoints
#define DELAY_TIME 10      // bandwidth of cntrl loop, controls the delay in loop(), smaller is better

//Note: negative error when less than desired position 0-1023
//int SET_A = 15; // first desired position (setpoint A)
//int SET_B = 54; // second desired position (setpoint B)
//int SET_C = 75; // third desired position (setpoint C)
//int SET_D = 90; // fourth desired position (setpoint D)
const float p_gain1 = 3;
const float p_gain = 0.2; // better performance, but makes overshoot .3, 0.006
const float d_gain = 0.5; // control overshoot .1, 0.00001

int iloop0 = 0;            // count no. of control loops
int setpoint0 = 20;     // init setpoint
int iloop1 = 0;            // count no. of control loops
int setpoint1 = 20;     // init setpoint
int iloop2 = 0;            // count no. of control loops
int setpoint2 = 20;     // init setpoint

unsigned long curr_time0;
unsigned long old_time0;
unsigned long dT0;
unsigned long t_flip0;
unsigned long t_report0;
float error0 = 0.0;
float old_error0 = 0.0;
int n_report0 = 0;
unsigned long curr_time1;
unsigned long old_time1;
unsigned long dT1;
unsigned long t_flip1;
unsigned long t_report1;
float error1 = 0.0;
float old_error1 = 0.0;
int n_report1 = 0;
unsigned long curr_time2;
unsigned long old_time2;
unsigned long dT2;
unsigned long t_flip2;
unsigned long t_report2;
float error2 = 0.0;
float old_error2 = 0.0;
int n_report2 = 0;

int mtr_fwd0 = 3;
int mtr_bwd0 = 11;
int pot_pin0 = A0;
int mtr_fwd1 = 5;
int mtr_bwd1 = 6;
int pot_pin1 = A1;
int mtr_fwd2 = 9;
int mtr_bwd2 = 10;
int pot_pin2 = A2;

const int output_low = -20;
const int output_high = 20;
const int motor_high = 250;
const int motor_low = 100;
const float motor_stop = 0.75;
const float motor_stop1 = 6.0;

bool finished0 = false;
bool finished1 = false;
bool finished2 = false;

int angle_of_distance = -1;

//ServoMotor motor0(6, 9, 1);
//ServoMotor motor1(10, 11, 2);
//ServoMotor motor2(3, 5, 0);

void setup()
{
  Serial.begin(9600);
  //
  //    //Sensor bootup
  //    Serial.println("Adafruit VL53L0X test:");
  //    if (!lox.begin()) {
  //      Serial.println(F("Failed to boot VL53L0X"));
  //      Serial.println("Boot failed.");
  //    }
  ////   wait until serial port opens for native USB devices
  //  while (! Serial) {
  //    delay(1);
  //  }
  //
  //  Serial.println("Adafruit VL53L0X test");
  //  if (!lox.begin()) {
  //    Serial.println(F("Failed to boot VL53L0X"));
  //    while (1);
  //  }
  //  // power
  //  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

  curr_time0 = millis();
  t_flip0 = curr_time0;
  t_report0 = curr_time0;
  curr_time1 = millis();
  t_flip1 = curr_time1;
  t_report1 = curr_time1;
  curr_time2 = millis();
  t_flip2 = curr_time2;
  t_report2 = curr_time2;

  //init the motor pins
  pinMode(mtr_fwd0, OUTPUT);
  pinMode(mtr_bwd0, OUTPUT);
  digitalWrite(mtr_fwd0, LOW);
  digitalWrite(mtr_bwd0, LOW);
  pinMode(mtr_fwd1, OUTPUT);
  pinMode(mtr_bwd1, OUTPUT);
  digitalWrite(mtr_fwd1, LOW);
  digitalWrite(mtr_bwd1, LOW);
  pinMode(mtr_fwd2, OUTPUT);
  pinMode(mtr_bwd2, OUTPUT);
  digitalWrite(mtr_fwd2, LOW);
  digitalWrite(mtr_bwd2, LOW);
}


void moveToSetpoint0(int setpoint) {
  iloop0++;  // increase loop counter value
  old_time0 = curr_time0;
  curr_time0 = millis();
  dT0 = curr_time0 - old_time0;  //time loop started-time from previous loop

  setpoint0 = setpoint;

  // update the position and the error
  int pos0 = analogRead(pot_pin0);
  pos0 = pos0 / 10; //take pos/10 to decrease the mapping error
  old_error0 = error0;
  error0 = pos0 - setpoint0;
  float dEdT0 = (float) (error0 - old_error0) / dT0;

  float output0 = p_gain * error0 + d_gain * dEdT0;
  output0 = constrain(output0, output_low, output_high);
  int val;
  if (output0 < -motor_stop) {
    // mtr_fwd
    val = map(output0, output_low, 0, 150, motor_high); //May need to change with weight
    analogWrite(mtr_fwd0, val);
    analogWrite(mtr_bwd0, LOW);
    finished0 = false;
  } else if (output0 > motor_stop) {
    // mtr_bwd
    val = map(output0, 0, output_high, 150, motor_high); //May need to change with weight
    analogWrite(mtr_fwd0, LOW);
    analogWrite(mtr_bwd0, val);
    finished0 = false;
  } else {
    // output is in a reasonable range, stop the engine
    analogWrite(mtr_fwd0, LOW);
    analogWrite(mtr_bwd0, LOW);
    finished0 = true;
  }

  delay(DELAY_TIME); //this delay controls the bandwidth of the controller

  // Update to serial port only once in a while since it will cause slow down
  if (curr_time0 - t_report0 > REPORT_TIME) {
    Serial.print("MOTOR 0--- ");
    Serial.print(" Control Bandwidth: "); Serial.print(1000.0 * (iloop0 - n_report0) / (curr_time0 - t_report0));
    Serial.print(" [Hz]; Pos: "); Serial.print(pos0);
    Serial.print("; Err: ");      Serial.print(error0);
    Serial.print("; dEdT: ");     Serial.print(dEdT0);
    Serial.print("; Out: ");      Serial.println(output0);
    Serial.print("; Motor Power: ");      Serial.println(val);
    t_report0 = curr_time0;
    n_report0 = iloop0;
  }
}

void moveToSetpoint1(int setpoint) {
  iloop1++;  // increase loop counter value
  old_time1 = curr_time1;
  curr_time1 = millis();
  dT1 = curr_time1 - old_time1;  //time loop started-time from previous loop

  setpoint1 = setpoint;

  // update the position and the error
  int pos1 = analogRead(pot_pin1);
  pos1 = pos1 / 10; //take pos/10 to decrease the mapping error
  old_error1 = error1;
  error1 = pos1 - setpoint;

  float dEdT1 = (float) (error1 - old_error1) / dT1;

  float output1 = p_gain1 * error1 + d_gain * dEdT1;
  output1 = constrain(output1, output_low, output_high);
  int val;
  if (output1 < -motor_stop1) {
    // mtr_fwd
    val = map(output1, output_low, 0, 150, motor_high); //May need to change with weight
    analogWrite(mtr_fwd1, val);
    analogWrite(mtr_bwd1, LOW);
    finished1 = false;
  } else if (output1 > motor_stop1) {
    // mtr_bwd
    val = map(output1, 0, output_high, 150, motor_high); //May need to change with weight
    analogWrite(mtr_fwd1, LOW);
    analogWrite(mtr_bwd1, val);
    finished1 = false;
  } else {
    // output is in a reasonable range, stop the engine
    analogWrite(mtr_fwd1, LOW);
    analogWrite(mtr_bwd1, LOW);
    finished1 = true;
  }

  delay(DELAY_TIME); //this delay controls the bandwidth of the controller

  // Update to serial port only once in a while since it will cause slow down
  if (curr_time1 - t_report1 > REPORT_TIME) {
    Serial.print("MOTOR 1--- ");
    Serial.print(" Control Bandwidth: "); Serial.print(1000.0 * (iloop1 - n_report1) / (curr_time1 - t_report1));
    Serial.print(" [Hz]; Pos: "); Serial.print(pos1);
    Serial.print("; Err: ");      Serial.print(error1);
    Serial.print("; dEdT: ");     Serial.print(dEdT1);
    Serial.print("; Out: ");      Serial.println(output1);
    Serial.print("; Motor Power: ");      Serial.println(val);
    t_report1 = curr_time1;
    n_report1 = iloop1;
  }
}

void moveToSetpoint2(int setpoint) {
  iloop2++;  // increase loop counter value
  old_time2 = curr_time2;
  curr_time2 = millis();
  dT2 = curr_time2 - old_time2;  //time loop started-time from previous loop

  setpoint2 = setpoint;

  // update the position and the error
  int pos2 = analogRead(pot_pin2);
  pos2 = pos2 / 10; //take pos/10 to decrease the mapping error
  old_error2 = error2;
  error2 = pos2 - setpoint;

  float dEdT2 = (float) (error2 - old_error2) / dT2;

  float output2 = p_gain * error2 + d_gain * dEdT2;
  output2 = constrain(output2, output_low, output_high);
  int val;
  if (output2 < -motor_stop) {
    // mtr_fwd
    val = map(output2, output_low, 0, motor_low, motor_high); //May need to change with weight
    analogWrite(mtr_fwd2, val);
    analogWrite(mtr_bwd2, LOW);
    finished2 = false;
  } else if (output2 > motor_stop) {
    // mtr_bwd
    val = map(output2, 0, output_high, motor_low, motor_high); //May need to change with weight
    analogWrite(mtr_fwd2, LOW);
    analogWrite(mtr_bwd2, val);
    finished2 = false;
  } else {
    // output is in a reasonable range, stop the engine
    analogWrite(mtr_fwd2, LOW);
    analogWrite(mtr_bwd2, LOW);
    finished2 = true;
  }

  delay(DELAY_TIME); //this delay controls the bandwidth of the controller

  // Update to serial port only once in a while since it will cause slow down
  if (curr_time2 - t_report2 > REPORT_TIME) {
    Serial.print("MOTOR 2--- ");
    Serial.print(" Control Bandwidth: "); Serial.print(1000.0 * (iloop2 - n_report2) / (curr_time2 - t_report2));
    Serial.print(" [Hz]; Pos: "); Serial.print(pos2);
    Serial.print("; Err: ");      Serial.print(error2);
    Serial.print("; dEdT: ");     Serial.print(dEdT2);
    Serial.print("; Out: ");      Serial.println(output2);
    Serial.print("; Motor Power: ");      Serial.println(val);
    t_report2 = curr_time2;
    n_report2 = iloop2;
  }
}

void loop()
{
//  finished1 = false;
//  while (!finished1) {
//    moveToSetpoint1(62); // used to be 83, 76
//  }
  
//  delay(500);
//  searchMode();
//  delay(500);
  //    while (angle_of_distance == -1) {
  //       searchMode();
  //    }
  //    Serial.print("Angle of distance: ");
  //    Serial.println(angle_of_distance);

  //    resetArmDown();
  finished1 = false;
  while (!finished1) {
    moveToSetpoint1(80); // used to be 83, 76
  }
  delay(1000);
  finished1 = false;
  callOver();
  delay(500);
  lowFive();
  //  crawlMode();
  delay(2000);
}

/*
   Start searching for the closest area. End on a rotation
   to the place that is closest.
*/
void searchMode() {
  Serial.println("Beginning Search");
  //Rotate motor0, bring motor1 and 3 to a semi lifted straight arm
  //  int closest_distance;
  //  //Rotate motor0, bring motor1 and 3 to a semi lifted straight arm
  //  if (millis() - curr_time0 > DELAY_TIME) {
  //    VL53L0X_RangingMeasurementData_t measure;
  //
  //    Serial.print("Reading a measurement... ");
  //    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  ////
  //    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  //    closest_distance = 9000;

  for (int i = 10; i <= 90; i++) {
    moveToSetpoint0(i);
    delay(10);
    //      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    //        if (measure.RangeMilliMeter < closest_distance) {
    //          closest_distance = measure.RangeMilliMeter;
    //          angle_of_distance = i;
    //        }
    //        Serial.println(i);
    //      } else {
    //        Serial.println(" out of range ");
  }
  //    }
  delay(500);
  //    moveToSetpoint0(angle_of_distance);
  moveToSetpoint0(54);
  //    curr_time0 = millis();
  //  }
}


/*
   Set the arm to be fully extended in preparation for a crawl
   or a search
*/
void resetArmDown() {
  Serial.println("RESET MODE:UP");
  while (!finished1) {
    moveToSetpoint2(46);
    delay(3);
    moveToSetpoint1(30); //slightly up // used to be 25
  }
  finished1 = false;
  finished2 = false;
  delay(500);

  Serial.println("RESET MODE:DOWN");
  while (!finished2) {
    moveToSetpoint1(55); //slightly up // used to be 25, 56
    delay(3);
    moveToSetpoint2(95); // used to be 83
  }
  finished2 = false;
  finished1 = false;
  delay(500);
}

void callOver() {
  Serial.println("CALL OVER");
  finished1 = false;
  while (!finished2) {
    moveToSetpoint2(53);
    delay(3);
  }
  finished2 = false;
  while (!finished2) {
    moveToSetpoint2(95); //slightly up // used to be 25
  }
  finished2 = false;
  while (!finished2) {
    moveToSetpoint2(53);
    delay(3);
  }
  finished2 = false;
  while (!finished2) {
    moveToSetpoint2(95); //slightly up // used to be 25
  }
  delay(500);
  angle_of_distance = -1;
}

void lowFive() {
  Serial.println("LOW FIVE");
  finished2 = false;
  while (!finished2) {
    moveToSetpoint2(53);
    //    delay(3);
    //    moveToSetpoint1(95); //slightly up // used to be 25
  }
  finished1 = false;
  while (!finished1) {
    moveToSetpoint1(95); //slightly up // used to be 25
  }
  finished1 = false;
  finished2 = false;
  angle_of_distance = -1;
}

/*
   Crawl in the direction that the arm is currently in.
*/
void crawlMode() {
  Serial.println("CRAWL MODE:DOWN");
  while (!finished2) {
    moveToSetpoint2(53);
    //    delay(3);
    //    moveToSetpoint1(95); //slightly up // used to be 25
  }
  while (!finished1) {
    moveToSetpoint1(95); //slightly up // used to be 25
  }
  finished1 = false;
  finished2 = false;
  delay(500);

  Serial.println("CRAWL MODE:UP");
  while (!finished2) {
    moveToSetpoint2(60); //slightly up // used to be 25, 56
    delay(3);
  }
  finished2 = false;
  while (!finished2) {
    moveToSetpoint2(50); //slightly up // used to be 25, 56
    delay(3);
  }
  while (!finished1) {
    moveToSetpoint1(62); // used to be 83, 76
  }

  finished2 = false;
  finished1 = false;
  delay(500);
  angle_of_distance = -1;
}
