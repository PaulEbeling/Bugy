//
// Created by Felix on 03.06.2023.
//

#include "gyro.h"

float fd;
std::vector<double> cali;

void gyro_init(){
    fd = wiringPiI2CSetup(0x68);
    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 128);
    wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01); //PLL with X axis gyroscope reference
    wiringPiI2CWriteReg8 (fd, CONFIG, 0); //Input disabled
    wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 0b00011000); //Full Scale Range +- 2000 °/s
    wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01); //Data Ready Interrupt

    cali = std::vector<double>(3);
    cali[0] = 0;
    cali[1] = 0;
    cali[2] = 0;
    calibration();
}

short read_raw_data(int addr){
    short high_byte,low_byte,value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr+1);
    value = (high_byte << 8) | low_byte;
    return value;
}

std::vector<double> get_xyz(){
    std::vector<double> xyz(3);

    xyz[0] = (read_raw_data(GYRO_XOUT_H) / 16.4) - cali[0];
    xyz[1] = (read_raw_data(GYRO_YOUT_H) / 16.4) - cali[1];
    xyz[2] = (read_raw_data(GYRO_ZOUT_H) / 16.4) - cali[2];

    return xyz;
}

void calibration(){
    std::vector<double> xyz(3);
    std::vector<double> xyz_temp(3);
    for (int i = 0; i < calibration_nr; i++) {
        xyz_temp = get_xyz();
        xyz[0] = xyz[0] + xyz_temp[0];
        xyz[1] = xyz[1] + xyz_temp[1];
        xyz[2] = xyz[2] + xyz_temp[2];
    }
    cali[0] = xyz[0]/calibration_nr;
    cali[1] = xyz[1]/calibration_nr;
    cali[2] = xyz[2]/calibration_nr;
}

