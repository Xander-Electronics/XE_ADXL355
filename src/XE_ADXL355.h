/*
  ADXL 355 Library for Arduino
  Copyright (c) 2022 Xander Electronics. All right reserved.
*/

#ifndef ADXL355_H
#define ADXL355_H

#include <Arduino.h>
#include <SPI.h>
#include <constants.h>

class XE_ADXL355Class {
public:
  bool begin(SPIClass &spi = SPI, int8_t drdyPin = -1, int8_t rstPin = -1, int8_t csPin = 10, uint32_t bitrate = 8000000);
  uint8_t getStatus();

  bool NVMBusy();
  bool activity();
  bool fifoOvr();
  bool fifoFull();
  bool isReady();

  float readTemperature();

  void setActivityAxes(uint8_t activityMask);
  void setActivityThreshold(uint16_t threshold);
  void setActivityCount(uint8_t count);
  void setActivityMask(uint8_t activityMask);

  void setRange(uint8_t range);

  void readAxes(float* measures);

  void start();
  void stop();

private:
  void write8(uint8_t thisRegister, uint8_t thisValue, bool writeInOr = false);
  void writeRegister(uint8_t thisRegister, uint8_t thisValue);
  
  uint8_t read8(uint8_t thisRegister);
  uint8_t readRegister(uint8_t thisRegister);

  float rawToValue(uint32_t raw);

  void readMultipleData(uint8_t thisRegister, uint8_t dataSize, uint8_t *data);

protected:
  SPISettings ADXL355Settings;
  SPIClass *_spi;
  int8_t _drdyPin, _rstPin, _csPin;
};

extern XE_ADXL355Class ADXL355;
#endif
