/*
  ADXL 355 Library for Arduino
  Copyright (c) 2022 Xander Electronics. All right reserved.
*/

#include "XE_ADXL355.h"

bool XE_ADXL355Class::begin(HardwareSPI &spi, int8_t drdyPin, int8_t rstPin, int8_t csPin, uint32_t bitrate) {
  _spi = &spi;

  if(drdyPin > 0) {
    _drdyPin = drdyPin;
    pinMode(_drdyPin, INPUT);
  }

  if(rstPin >= 0) {
    _rstPin = rstPin;
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, LOW);
    delay(100);
    digitalWrite(rstPin, HIGH);
    delay(100);
  }

  if(_csPin >= 0) {
    _csPin = csPin;
    pinMode(_csPin, OUTPUT);
  }

  ADXL355Settings = SPISettings(bitrate, MSBFIRST, SPI_MODE0);
  _spi->begin();
  uint8_t id = read8(ADXL355_PARTID);

  if (id != ADXL355_ID) {
    return false;
  } else {
    return true;
  }
}

uint8_t XE_ADXL355Class::getStatus(void) {
  return readRegister(ADXL355_STATUS);
}

bool XE_ADXL355Class::NVMBusy(void) {
  uint8_t status = getStatus();

  if (status & (1 << STATUS_NVMBUSY_POS)) {
    return true;
  } else {
    return false;
  }
}

bool XE_ADXL355Class::activity(void) {
  uint8_t status = getStatus();

  if (status & (1 << STATUS_ACTIVITY_POS)) {
    return true;
  } else {
    return false;
  }
}

bool XE_ADXL355Class::fifoOvr(void) {
  uint8_t status = getStatus();

  if (status & (1 << STATUS_FIFOVR_POS)) {
    return true;
  } else {
    return false;
  }
}

bool XE_ADXL355Class::fifoFull(void) {
  uint8_t status = getStatus();

  if (status & (1 << STATUS_FIFOFULL_POS)) {
    return true;
  } else {
    return false;
  }
}

bool XE_ADXL355Class::isReady(void) {
  uint8_t status = getStatus();

  if (status & (1 << STATUS_DATARDY_POS)) {
    return true;
  } else {
    return false;
  }
}

float XE_ADXL355Class::readTemperature() {
  uint8_t temperatureArray[] = {0, 0};
  readMultipleData(ADXL355_TEMP2, 2, temperatureArray);
  float t = ((temperatureArray[0] - ((temperatureArray[0] >> 4) << 4)) << 8) + temperatureArray[1];
  t = -(t - 1885.0) / 9.05 + 25.0 - 8.5;
  return t;
}

void XE_ADXL355Class::setActivityMask(uint8_t activityMask) {
  write8(ADXL355_ACT_EN, activityMask, true);
}

void XE_ADXL355Class::setActivityThreshold(uint16_t threshold) {
  write8(ADXL355_ACT_THRESH_H, (threshold >> 8) & 0xFF);
  write8(ADXL355_ACT_THRESH_L, (threshold) & 0xFF);
}

void XE_ADXL355Class::setActivityCount(uint8_t count) {
  write8(ADXL355_ACT_COUNT, count);
}

void XE_ADXL355Class::setRange(uint8_t range) {
  write8(ADXL355_RANGE, range & 0x0F, true);
}

void XE_ADXL355Class::start() {
  write8(ADXL355_POWER_CTL, 0x01);
}

void XE_ADXL355Class::stop() {
  write8(ADXL355_POWER_CTL, 0);
}

void XE_ADXL355Class::write8(uint8_t thisRegister, uint8_t thisValue, bool writeInOr) {
  digitalWrite(_csPin, LOW);
  _spi->beginTransaction(ADXL355Settings);
  if (writeInOr) {
    uint8_t value = readRegister(thisRegister) | thisValue;
    writeRegister(thisRegister, value);
  } else {
    writeRegister(thisRegister, thisValue);
  }
  _spi->endTransaction();
  digitalWrite(_csPin, HIGH);
}

void XE_ADXL355Class::writeRegister(uint8_t thisRegister, uint8_t thisValue) {
  _spi->transfer((thisRegister << 1) | WRITE_BYTE);
  _spi->transfer(thisValue);
}

uint8_t XE_ADXL355Class::read8(uint8_t thisRegister) {
  uint8_t result = 0;
  digitalWrite(_csPin, LOW);
  _spi->beginTransaction(ADXL355Settings);
  result = readRegister(thisRegister);
  _spi->endTransaction();
  digitalWrite(_csPin, HIGH);
  return result;
}

uint8_t XE_ADXL355Class::readRegister(uint8_t thisRegister) {
  _spi->transfer((thisRegister << 1) | READ_BYTE);
  return _spi->transfer(0x00);
}

void XE_ADXL355Class::readMultipleData(uint8_t thisRegister, uint8_t dataSize, uint8_t *data) {
  digitalWrite(_csPin, LOW);
  _spi->beginTransaction(ADXL355Settings);
  SPI.transfer((thisRegister << 1) | READ_BYTE);
  for (int i = 0; i < dataSize; i++) {
    data[i] = _spi->transfer(0x00);
  }
  _spi->endTransaction();
  digitalWrite(_csPin, HIGH);
}

void XE_ADXL355Class::readAxes(float* measures) {
  uint8_t axesMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  readMultipleData(ADXL355_XDATA3, 9, axesMeasures);

  uint32_t xdata = ((uint32_t) axesMeasures[2] >> 4) | ((uint32_t) axesMeasures[1] << 4) | ((uint32_t) axesMeasures[0] << 12);
  uint32_t ydata = ((uint32_t) axesMeasures[5] >> 4) | ((uint32_t) axesMeasures[4] << 4) | ((uint32_t) axesMeasures[3] << 12);
  uint32_t zdata = ((uint32_t) axesMeasures[8] >> 4) | ((uint32_t) axesMeasures[7] << 4) | ((uint32_t) axesMeasures[6] << 12);

  measures[0] = rawToValue(xdata);
  measures[1] = rawToValue(ydata);
  measures[2] = rawToValue(zdata);
}

float XE_ADXL355Class::rawToValue(uint32_t raw) {
  float x = (float) raw;
  
  x = raw > 0X7FFFF ? x - 0XFFFFE : x;
  x = x / 0X7FFFF * 2.048;

  return x;
}
XE_ADXL355Class ADXL355;