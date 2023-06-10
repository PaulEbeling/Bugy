cmake_minimum_required(VERSION 3.20)
project(Bugy)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(Bugy
        adafruitdcmotor.cpp
        adafruitdcmotor.h
        adafruitmotorhat.cpp
        adafruitmotorhat.h
        gyro.cpp
        gyro.h
        i2cdevice.cpp
        i2cdevice.h
        main.cpp
        pwm.cpp
        pwm.h
        util.h)
