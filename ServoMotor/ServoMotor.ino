#include "ServoMotor.h"

ServoMotor motor0(6, 9, 1);
ServoMotor motor1(10, 11, 2);
ServoMotor motor2(3, 5, 0);

void setup()
{
  Serial.begin(9600);
  //Motor 1
  motor0.setupMotor();
  motor0.setPID(0.2, 0, 0.5);
//  motor0.setSetpoint(50);

  //Motor 2
  motor1.setupMotor();
  motor1.setPID(0.2, 0, 0.5);
//  motor1.setSetpoint(50);

  //Motor 3
  motor2.setupMotor();
  motor2.setPID(0.2, 0, 0.5);
//  motor2.setSetpoint(50);
}

void loop()
{
  resetArmDown();
  delay(2000);
}

/*
   Start searching for the closest area. End on a rotation
   to the place that is closest.
*/
void searchMode() {
  //Rotate motor0, bring motor1 and 3 to a semi lifted straight arm
}


/*
   Set the arm to be fully extended in preparation for a crawl
   or a search
*/
void resetArmDown() {
  motor1.mtr_fwd = 10;
  motor1.mtr_bwd = 11;
  motor1.pot_pin = 2;
  motor1.setSetpoint(54); //45d
  motor1.moveToSetpoint();
  delay(1000);
  
  motor0.mtr_fwd = 6;
  motor0.mtr_bwd = 9;
  motor0.pot_pin = 1;
  motor0.setSetpoint(25);  //"0"d
  motor0.moveToSetpoint();
 
  delay(1000);
//  motor1.setSetpoint(81); //135d
//  motor1.moveToSetpoint();
//  motor2.setSetpoint(54); //135d
//  motor2.moveToSetpoint();
//  delay(1000);
}

//unsigned long moveToSetpoint(unsigned long prev_loop_time, int pot_pin, float error) {
//    old_time = prev_loop_time;
//    curr_time = millis();
//    dT = curr_time - old_time;  //time loop started-time from previous loop
//
//    // update the position and the error
//    int pos = analogRead(pot_pin);
//    pos = pos / 10; //take pos/10 to decrease the mapping error
//    old_error = error;
//    error = pos - setpoint;
//    float dEdT = (float) (error - old_error) / dT;
//
//    float output = p_gain * error + d_gain * dEdT;
//    output = constrain(output, output_low, output_high);
//    if (output < -motor_stop) {
//      // mtr_fwd
//      int val = map(output, output_low, 0, motor_low, motor_high); //May need to change with weight
//      analogWrite(mtr_fwd, val);
//      analogWrite(mtr_bwd, LOW);
//    } else if (output > motor_stop) {
//      // mtr_bwd
//      int val = map(output, 0, output_high, motor_low, motor_high); //May need to change with weight
//      analogWrite(mtr_fwd, LOW);
//      analogWrite(mtr_bwd, val);
//    } else {
//      // output is in a reasonable range, stop the engine
//      analogWrite(mtr_fwd, LOW);
//      analogWrite(mtr_bwd, LOW);
//      finished = true;
//    }
//
//    // Update to serial port only once in a while since it will cause slow down
//    if (millis() - curr_time > REPORT_TIME) {
//      Serial.print("Pos: "); Serial.print(pos);
//      Serial.print("; Err: ");      Serial.print(error);
//      Serial.print("; dEdT: ");     Serial.print(dEdT);
//      Serial.print("; Out: ");      Serial.println(output);
//      curr_time = millis();
//    }
//    return curr_time;
//}

/*
   Crawl in the direction that the arm is currently in.
*/
void crawlMode() {
  //Reset to grab the ground
  resetArmDown();
}
