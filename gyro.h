//
// Created by Felix on 03.06.2023.
//

#ifndef BUGGY_GYRO_H
#define BUGGY_GYRO_H


#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <wiringPiI2C.h>

#define Device_Address 0x68

#define PWR_MGMT_1   0x6B
#define SMPRT_DIV    0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE   0x38

#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

#define calibration_nr 1000

void gyro_init();

short read_raw_data(int addr);

std::vector<double> get_xyz();

void calibration();

#endif //BUGGY_GYRO_H
