#include "Adafruit_VL53L0X.h"

//#define DELAY_TIME 100
Adafruit_VL53L0X lox; //= Adafruit_VL53L0X();
unsigned long curr_time;


void setup() {
  Serial.begin(9600);
  curr_time = millis();

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");
  lox.begin();
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
}


void loop() {
  //if (millis() - curr_time > DELAY_TIME) {
//    VL53L0X_RangingMeasurementData_t measure;
//
//    Serial.print("Reading a measurement... ");
//    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
//
//    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
//    } else {
//      Serial.println(" out of range ");
//    }
//    curr_time = millis();
//  //}
    VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}
