/*
  ADXL 355 Library for Arduino
  Copyright (c) 2022 Xander Electronics. All right reserved.

  Simple: Read X, Y, Z and Temperature using interrupt
*/

#include <XE_ADXL355.h>

void setup() {
  Serial.begin(115200);
  while(!Serial);

  bool initialized = ADXL355.begin(SPI, -1, -1, 2, 1000000UL); //SPI to use, drdy pin, reset pin, cs, pin, bitrate

  if(!initialized) {
    Serial.println("Failed to initialize ADXL355");
    while(1);
  }

  ADXL355.setRange(RANGE_2G);

  ADXL355.setActivityAxes(ACTIVITY_X | ACTIVITY_Y | ACTIVITY_Z); //detect activity on X, Y or Z
  ADXL355.setActivityCount(1); //generate interrupt after each event 
  ADXL355.setActivityThreshold(127);  //1g
  ADXL355.setActivityThreshold(-127); //-1g
  ADXL355.enableInterrupt(INT_ACTIVITY, 10, INT1, FALLING); //enable interrupt on activity through INT1 of ADXL355
  ADXL355.start();
}

void loop() {
  
  if (ADXL355.isReady()) {
    Serial.println("Temp = " + String(ADXL355.readTemperature()) + " *C");
    float data[] = {0, 0, 0};
    ADXL355.readAxes(data);
    Serial.println("x = " + String(data[0]) + ", y = " + String(data[1]) + ", z = " + String(data[2]));
  }
}
