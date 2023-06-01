///
/// Project main
///

/// Includes
#include <signal.h>
#include <iostream>
#include <thread>
#include <chrono>
#include "source/adafruitmotorhat.h"
#include <wiringPi.h>

/// Interrupt Routine for STRG-C
void signalHandler(int signum)
{
    std::cout << "Strg-C Programmende" << std::endl;
	// Beenden Sie hier bitte alle Verbindung zu den Sensoren etc.
    exit(signum);
}


int test()
{
    using namespace std::chrono_literals;

    // connect using the default device address 0x60
    AdafruitMotorHAT hat;

    // get the motor connected to port 1
    if (auto motor { hat.getMotor (1) })
    {
        // speed must be set before running commands
        motor->setSpeed (255);

        motor->run (AdafruitDCMotor::kForward);
        std::this_thread::sleep_for (1s);

        motor->run (AdafruitDCMotor::kBackward);
        std::this_thread::sleep_for (1s);

        // release the motor after use
        motor->run (AdafruitDCMotor::kRelease);
    }

    return 0;
}

int main()
{
    // Csignal für Abbruch über STRG-C
    signal(SIGINT, signalHandler);

    test();
    return 0;
}
