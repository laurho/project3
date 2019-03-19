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
  motor0.setSetpoint(50);

  //Motor 2
  motor1.setupMotor();
  motor1.setPID(0.2, 0, 0.5);
  motor1.setSetpoint(50);

  //Motor 3
  motor2.setupMotor();
  motor2.setPID(0.2, 0, 0.5);
  motor2.setSetpoint(50);
}

void loop()
{
  resetArmDown();
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
  motor1.setSetpoint(54); //45d
  motor1.moveToSetpoint();
//  motor2.setSetpoint(5);  //"0"d
//  motor2.moveToSetpoint();
//  motor1.setSetpoint(81); //135d
//  motor1.moveToSetpoint();
}


/*
   Crawl in the direction that the arm is currently in.
*/
void crawlMode() {
  //Reset to grab the ground
  resetArmDown();
}
