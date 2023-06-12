///
/// Project main
///

/// Includes
#include <signal.h>
#include <iostream>
#include <chrono>
#include "adafruitmotorhat.h"
#include <wiringPi.h>
#include "gyro.h"
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#define motor_nr1 1
#define motor_nr2 4
#define speed 110
#define correction_speed 90
#define echo 3 //wiringpi port for echo
#define trigger 4 //wiringpi port for trigger
#define led 5 //wiringpi port for led
#define limit 10 //limit, how near the border is allowed to be in cm
#define correction .2 //the correction for driving straight

AdafruitMotorHAT hat;
std::shared_ptr <AdafruitDCMotor> motor;
std::shared_ptr <AdafruitDCMotor> motor2;
int gamepad;


int init();

double distance();

void straight(double milli_sec = 0, struct js_event *event = nullptr);

void left(bool left = true, float degree = 90, bool manuel = false);

void rectangle();

void bypass();

void controller();

int main();

/// Interrupt Routine for STRG-C
void signalHandler(int signum) {
    std::cout << "Strg-C Programmende" << std::endl;
    // Beenden Sie hier bitte alle Verbindung zu den Sensoren etc.

    motor->run(AdafruitDCMotor::kRelease);
    motor2->run(AdafruitDCMotor::kRelease);

    exit(signum);
}


bool controller_test(int btn) {
    struct js_event event;
    ssize_t bytesRead = read(gamepad, &event, sizeof(event));
    if (event.type == JS_EVENT_BUTTON) {
        int button = event.number;
        int value = event.value;
        std::cout << "Bin in haupt Button:  " << button << "  Value:  " << value << "\n";

        if (button == btn)
            if (value == 0) {
                return false;
            }
    }
    return true;
}


/**
 * Drives a straight line with object recognition
 * @param milli_sec Duration how long it should drive forward, in ms
 */
void straight(double milli_sec, struct js_event *alt) {
    //initialize vectors for xyz for movement correction

    std::vector<double> xyz(3);
    std::vector<double> xyz2(3);
    xyz2[0] = 0;
    xyz2[1] = 0;
    xyz2[2] = 0;

    unsigned long pre_time = micros(), aft_time = micros(), last_time = 0; //initialize time
    motor->run(AdafruitDCMotor::kForward);
    motor2->run(AdafruitDCMotor::kBackward);
    double time = millis() + milli_sec;

    double z = 0;
    bool controller_break = true;
    while ((milli_sec != 0 ? time - millis() > 0 : true) && controller_break) {
        xyz = get_xyz();
        if (milli_sec == 0 && !controller_test(3)) controller_break = false;
        if (distance() <= limit) {

            motor->run(AdafruitDCMotor::kRelease);
            motor2->run(AdafruitDCMotor::kRelease);
            bypass();
            motor->run(AdafruitDCMotor::kForward);
            motor2->run(AdafruitDCMotor::kBackward);
        }

        if (z > correction || z < -correction) {
            //motor->run(AdafruitDCMotor::kRelease);
            //motor2->run(AdafruitDCMotor::kRelease);
            //motor->setSpeed(correction_speed);
            //motor2->setSpeed(correction_speed);

            double old_z = z;

            while (old_z > correction ? z > correction : z < -correction && controller_break) {
                xyz = get_xyz();
                if (milli_sec == 0 && !controller_test(3)) controller_break = false;
                z > correction ? motor2->setSpeed(correction_speed) :
                motor->setSpeed(correction_speed);
                delay(2);
                z = z + ((last_time) * (xyz2[2] + xyz[2])) / (2000000); //calculate angle like in left()
                //std::cout << 0 << "  " << 0 << "  " << z << "  " << last_time << "while \n";
                aft_time = micros(); //stop "timer"
                last_time = aft_time - pre_time; // calculate time
                pre_time = micros(); //start "timer"
                xyz2 = get_xyz();
                //motor->run(AdafruitDCMotor::kRelease);
                //motor2->run(AdafruitDCMotor::kRelease);
            }

            motor->run(AdafruitDCMotor::kForward);
            motor2->run(AdafruitDCMotor::kBackward);
            motor->setSpeed(speed);
            motor2->setSpeed(speed);
        }
        z = z + ((last_time) * (xyz2[2] + xyz[2])) / (2000000); //calculate angle like in left()
        //std::cout << 0 << "  " << 0 << "  " << z << "  " << last_time << "\n";
        aft_time = micros(); //stop "timer"
        last_time = aft_time - pre_time; // calculate time
        pre_time = micros(); //start "timer"
        xyz2 = get_xyz();
    }

    motor->run(AdafruitDCMotor::kRelease);
    motor2->run(AdafruitDCMotor::kRelease);
}

/**
 * Dirves an 90 degree angle
 * @param left true it drives left round; false it is moving right round
 */
void left(bool left, float degree, bool manuel) {
    //initialize vectors for xyz
    std::vector<double> xyz(3);
    std::vector<double> xyz2(3);
    xyz2[0] = 0;
    xyz2[1] = 0;
    xyz2[2] = 0;

    unsigned long pre_time = micros(), aft_time = micros(), last_time = 0; //initialize time
    motor->run(left ? AdafruitDCMotor::kBackward : AdafruitDCMotor::kForward);

    if (manuel) {
        double z = 0;
        while (true) {
            if(!controller_test(2) || !controller_test(1)) {
                std::cout << "hahhahahh";
                break;
            }

            xyz = get_xyz();/*
            x = x + ((last_time) * (xyz2[0] + xyz[0])) / (2000000);
            y = y + ((last_time) * (xyz2[1] + xyz[1])) / (2000000);*/
            z = z + ((last_time) * (xyz2[2] + xyz[2])) / (2000000); //calculate angle: adding last and current z
            // axes
            // divided by 1000000 because of microseconds and * 2 because area of parallelogram
            //std::cout << x << "  " << y << "  " << z << "  " << last_time << "\n";
            aft_time = micros(); //stop "timer"
            last_time = aft_time - pre_time; // calculate time
            pre_time = micros(); //start "timer"
            xyz2 = get_xyz();
        }
    } else {
        //run motors until 84 degrees are reached; 84 -> because it works better, motors need timer to stop
        for (double/* x = 0, y = 0,*/ z = 0; z < degree - 6 && z > -degree + 6;) {

            xyz = get_xyz();/*
        x = x + ((last_time) * (xyz2[0] + xyz[0])) / (2000000);
        y = y + ((last_time) * (xyz2[1] + xyz[1])) / (2000000);*/
            z = z + ((last_time) * (xyz2[2] + xyz[2])) / (2000000); //calculate angle: adding last and current z axes
            // divided by 1000000 because of microseconds and * 2 because area of parallelogram
            //std::cout << x << "  " << y << "  " << z << "  " << last_time << "\n";
            aft_time = micros(); //stop "timer"
            last_time = aft_time - pre_time; // calculate time
            pre_time = micros(); //start "timer"
            xyz2 = get_xyz();
        }
    }
    motor->run(AdafruitDCMotor::kBrake);
}


/**
 * Drives a square with collision detection
 */
void rectangle() {
    for (int i = 0; i <= 4; i++) {
        straight(3000);
        left();
    }
}

/**
 * Measures the distance to an object
 * @return distance in cm
 */
double distance() {
    digitalWrite(trigger, HIGH);
    delay(0.01);
    digitalWrite(trigger, LOW);
    while (digitalRead(echo) == LOW) {}
    double time = micros(); //start timer
    while (digitalRead(echo) == HIGH) {}
    time = micros() - time;
    //std::cout << time * 0.017  << "  " << time << "\n";

    double dist = time * 0.017; // 340m/s * 100 -> 34000 in cm -> / 2 both directions -> /1000000
    dist <= limit ? digitalWrite(led, HIGH) : digitalWrite(led, LOW);
    return dist;
}

/**
 * Initializes gyro Sensor and sets global variables for the motors and sets inputs/outputs for led and ultrasonic
 * blaster
 */
int init() {
    gyro_init();

    //motor
    motor = hat.getMotor(motor_nr1);
    motor2 = hat.getMotor(motor_nr2);
    motor->setSpeed(speed);
    motor2->setSpeed(speed);

    //ultrasonic blaster
    wiringPiSetup();
    pinMode(echo, INPUT);
    pinMode(trigger, OUTPUT);

    //led
    pinMode(led, OUTPUT);

    //Controller
    // Open the Gamepad
    gamepad = open("/dev/input/js0", O_NONBLOCK);

    if (gamepad == -1) {
        std::cerr << "Gamepad could not be opened." << std::endl;
        return 1;
    }
    return 0;
}

/**
 * Bypasses an Object by moving left (if looking ahead it wont work, it will go on with right, if right
 * to short it will go on with another bypass)
 * another bypass execution)then forward and
 */
void bypass() {
    left();
    bool left_curve = true;
    if (distance() < 50) {
        left(false);
        left(false);
        left_curve = false;
    }

    straight(4000);
    left_curve ? left(false) : left();
    straight(6000);
    left_curve ? left(false) : left();
    straight(4000);
    left_curve ? left() : left(false);
}

/**
 * Tests for Controller inputs
 */
void controller() {
    // Hauptprogrammschleife
    while (true) {
        // Lesen Sie die Gamepad-Ereignisse
        struct js_event event;
        ssize_t bytesRead = read(gamepad, &event, sizeof(event));

        /*
        if (bytesRead == -1) {
            std::cerr << "Fehler beim Lesen des Gamepad-Ereignisses." << std::endl;
        }*/

        if (bytesRead == sizeof(event)) {
            // Überprüfen Sie den Ereignistyp
            if (event.type == JS_EVENT_AXIS) {
                // Achsenwert erhalten
                int axis = event.number;
                int value = event.value;
                std::cout << "Achse " << axis << " Wert: " << value << std::endl;

                if (axis == 0 || axis == 2) {
                    // X Achse
                    // value is a signed integer between -32767 and +32767 representing the position of the joystick along that axis
                } else if (axis == 1 || axis == 3) {
                    // Y Achse
                    // value is a signed integer between -32767 and +32767 representing the position of the joystick along that axis
                }

                if (axis == 0) {
                    //  straight(100);
                }
            } else if (event.type == JS_EVENT_BUTTON) {
                int button = event.number;
                int value = event.value;
                std::cout << "Testtss" << button << "  " << value << "\n";
                switch (button) {
                    case 0:
                        if (value == 1) {
                            // A
                            straight(10);

                            std::cout << "Knopf Button 0 Value 1";
                        } else {

                        }
                        break;
                    case 1:
                        if (value == 1) {
                            // B
                            left(false, 0.0, true);
                            std::cout << "Knopf Button 1 Value 1";
                        } else {

                        }
                        break;
                    case 2:
                        if (value == 1) {
                            // X
                            left(true, 0.0, true);
                            std::cout << "Knopf Button 2 Value 1";
                        } else {

                        }
                        break;
                    case 3:
                        if (value == 1) {
                            // Y
                            std::cout << "Knopf Button 3 Value 1";
                            straight(0, &event);
                        } else {

                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }

    // Schließen Sie das Gamepad-Gerät
    close(gamepad);
}


int main() {
    // Csignal für Abbruch über STRG-C
    signal(SIGINT, signalHandler);
    if (init() == 0) {
        std::cout << "Please make Controller input";
        controller();
    }


    straight(5000);
    std::cout << "Please make input.";
    char input;
    for (;;) {
        input = getchar();
        switch (input) {
            case 'w':
                straight(100);
                break;
            case 'a':
                left();
                break;
            case 'd':
                left(false);
                break;
            case '.':
                signalHandler(5);
            default:
                std::cout << "Please make an other input! Or Terminate with 'STRG' + 'c'";
        }
    }

    return 0;
}
