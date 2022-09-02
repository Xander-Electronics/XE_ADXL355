/*
  ADXL 355 Library for Arduino
  Copyright (c) 2022 Xander Electronics. All right reserved.
*/

#ifndef ADXL355_CONSTANTS_H
#define ADXL355_REGISTERS_H

#define READ_BYTE                   0x01
#define WRITE_BYTE                  0x00

#define ADXL355_DEVID_AD            0x00
#define ADXL355_DEVID_MST           0x01
#define ADXL355_PARTID              0x02
#define ADXL355_REVID               0x03
#define ADXL355_STATUS              0x04
#define ADXL355_FIFO_ENTRIES        0x05
#define ADXL355_TEMP2               0x06
#define ADXL355_TEMP1               0x07
#define ADXL355_XDATA3              0x08
#define ADXL355_XDATA2              0x09
#define ADXL355_XDATA1              0x0A
#define ADXL355_YDATA3              0x0B
#define ADXL355_YDATA2              0x0C
#define ADXL355_YDATA1              0x0D
#define ADXL355_ZDATA3              0x0E
#define ADXL355_ZDATA2              0x0F
#define ADXL355_ZDATA1              0x10
#define ADXL355_FIFO_DATA           0x11
#define ADXL355_OFFSET_X_H          0x1E
#define ADXL355_OFFSET_X_L          0x1F
#define ADXL355_OFFSET_Y_H          0x20
#define ADXL355_OFFSET_Y_L          0x21
#define ADXL355_OFFSET_Z_H          0x22
#define ADXL355_OFFSET_Z_L          0x23
#define ADXL355_ACT_EN              0x24
#define ADXL355_ACT_THRESH_H        0x25
#define ADXL355_ACT_THRESH_L        0x26
#define ADXL355_ACT_COUNT           0x27
#define ADXL355_FILTER              0x28
#define ADXL355_FIFO_SAMPLES        0x29
#define ADXL355_INT_MAP             0x2A
#define ADXL355_SYNC                0x2B
#define ADXL355_RANGE               0x2C
#define ADXL355_POWER_CTL           0x2D
#define ADXL355_SELFTEST            0x2E

#define ADXL355_ID                  0xED
#define RANGE_2G                    0x01
#define RANGE_4G                    0x02
#define RANGE_8G                    0x03

#define STATUS_NVMBUSY_POS          4
#define STATUS_ACTIVITY_POS         3
#define STATUS_FIFOVR_POS           2
#define STATUS_FIFOFULL_POS         1
#define STATUS_DATARDY_POS          0

#define ACTIVITY_X                  0
#define ACTIVITY_Y                  1
#define ACTIVITY_Z                  2

#define ACT_EN2_POS                 7
#define OVR_EN2_POS                 6
#define FULL_EN2_POS                5
#define RDY_EN2_POS                 4
#define ACT_EN1_POS                 3
#define OVR_EN1_POS                 2
#define FULL_EN1_POS                1
#define RDY_EN1_POS                 0

#define INT_ACTIVITY                3
#define INT_OVERFLOW                2
#define INT_FIFOFULL                1
#define INT_DATAREADY               0

#define INT1                        0
#define INT2                        1
#endif
