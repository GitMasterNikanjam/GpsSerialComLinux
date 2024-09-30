// For compile and build: g++ -o ex2 ex2.cpp ../RaspGPS_UART.cpp ../../TinyGPS++/TinyGPS++.cpp -lpigpio -lpthread -Wall -Wextra -std=c++17
// For run: sudo ./ex2

// This example show use RaspGPS_UART with multithread strategy.

// ####################################################
// Include libraries:

#include <iostream>
#include <chrono>
#include "../RaspGPS_UART.h"

using namespace std;

// #####################################################
// Define parameters:

#define GPS_PPS_PIN             18          // GPIO 18, DOWN, PCM_CLK, SD10, DPI_D14, SPI6_CE0_N, SPI1_CE0_N, PWM0_0
#define BAUDRATE                115200      // UART baudrate for GPS communication

// #######################################################
// Define global variables and objects:

// GPS object 
RaspGPS_UART gps;

// for time storing.
uint64_t t1,t2;         

// Time origin point.
chrono::time_point<std::chrono::system_clock> T_origin;

// #######################################################
// Function declerations:

// Calculate time from origin time point in microseconds.
uint64_t micros(void);

// main loop()
void loop(void);

// ######################################################
// main:

int main(void)
{   
    // Save time point at object constructor.
    T_origin = chrono::high_resolution_clock::now();

    // Initial pigpio library.
    if (gpioInitialise() < 0) 
    {
        printf("pigpio initialization failed");
        return false;
    }

    gps.setPPSpin(GPS_PPS_PIN);
    gps.setBaudrate(BAUDRATE);

    if(gps.begin())
    {
        printf("GPS object inited and setup finished.\n");
    }
    else
    {
        printf("%s\n",gps.errorMessage.c_str());
    }

    gps.startThread();

    loop();

    // Close all resources of gps object.
    gps.clean();

    // Cleanup before exiting. This function is built in pigpio library
    gpioTerminate(); 

    return 0;
}

// #######################################################
// Functions:

void loop(void)
{
    while(true)
    {
        t1 = micros();
        double time[6];
        float pos[3];

        time[0] = gps.data.utcTime[0];
        time[1] = gps.data.utcTime[1];
        time[2] = gps.data.utcTime[2];
        time[3] = gps.data.utcTime[3];
        time[4] = gps.data.utcTime[4];
        time[5] = gps.data.utcTime[5];

        pos[0] = gps.data.lat;
        pos[1] = gps.data.lon;
        pos[2] = gps.data.alt;

        t2 = micros();

        printf("UTC time: ");
        std::cout << time[0] << ", " << time[1] << ", " << time[2] << ", " << time[3] << ", " << time[4] << ", " << time[5] <<  std::endl;

        printf("Location: ");
        std::cout << pos[0] << ", " << pos[1] << ", " << pos[2] <<std::endl;

        printf("Hdop: "); 
        std::cout << gps.data.hdop <<std::endl;

        printf("Fixed: "); 
        std::cout << gps.data.fixed <<std::endl;

        printf("duration: "); 
        std::cout << t2-t1 <<std::endl << std::endl;

        gpioDelay(1000);
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint64_t micros(void)
{
    std::chrono::time_point<std::chrono::system_clock> T_point;
    // Get Current time point
    T_point = std::chrono::high_resolution_clock::now();
    // Calculate Time duration
    auto T_dur = std::chrono::duration_cast<std::chrono::microseconds>(T_point - T_origin);
    // Return time duration in uint64_t 
    return static_cast<uint64_t>(T_dur.count());
}