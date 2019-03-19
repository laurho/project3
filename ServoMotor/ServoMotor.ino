#include "ServoMotor.h"

ServoMotor motor(6,9,1);
ServoMotor motor2(3,5,0);
ServoMotor motor3(10,11,2);

void setup()
{
  Serial.begin(9600);
  //Motor 1
  motor.setupMotor();
  motor.setPID(0.2, 0, 0.5);
  motor.setSetpoints(50, 90);

  //Motor 2
  motor2.setupMotor();
  motor2.setPID(0.2, 0, 0.5);
  motor2.setSetpoints(50, 90);

  //Motor 3
  motor3.setupMotor();
  motor3.setPID(0.2, 0, 0.5);
  motor3.setSetpoints(50, 90);
}

void loop()
{
  motor.loopMotor();
  motor2.loopMotor();
  motor3.loopMotor();
}
