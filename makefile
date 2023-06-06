OBJS	= adafruitdcmotor.o adafruitmotorhat.o i2cdevice.o main.o pwm.o gyro.o
SOURCE	= adafruitdcmotor.cpp adafruitmotorhat.cpp i2cdevice.cpp main.cpp pwm.cpp gyro.cpp
HEADER	= adafruitdcmotor.h adafruitmotorhat.h i2cdevice.h pwm.h util.h gyro.h
OUT	= main
CC	 = g++
FLAGS	 = -g -c -Wall
LFLAGS	 =

all: $(OBJS)
	$(CC) -g $(OBJS) -o $(OUT) $(LFLAGS) -lwiringPi

adafruitdcmotor.o: adafruitdcmotor.cpp
	$(CC) $(FLAGS) adafruitdcmotor.cpp

adafruitmotorhat.o: adafruitmotorhat.cpp
	$(CC) $(FLAGS) adafruitmotorhat.cpp

i2cdevice.o: i2cdevice.cpp
	$(CC) $(FLAGS) i2cdevice.cpp

main.o: main.cpp
	$(CC) $(FLAGS) -lwiringPi main.cpp

pwm.o: pwm.cpp
	$(CC) $(FLAGS) pwm.cpp

gyro.o: gyro.cpp
	$(CC) $(FLAGS) gyro.cpp

clean:
	rm -f $(OBJS) $(OUT)
