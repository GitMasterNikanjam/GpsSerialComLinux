/*
For compile and build for raspberry pi: 
mkdir -p ./bin && g++ -o ./bin/ex1 ex1.cpp ../GpsSerialComLinux.cpp ../../TinyGPSPlus_OS/TinyGPSPlus.cpp -lpigpio -lpthread -Wall -Wextra -std=c++17

For compile and build for General Linux OS: 
mkdir -p ./bin && g++ -o ./bin/ex1 ex1.cpp ../GpsSerialComLinux.cpp ../../TinyGPSPlus_OS/TinyGPSPlus.cpp -lpthread -Wall -Wextra -std=c++17
*/

// For run: sudo ./bin/ex1

// This example show use RaspGPS_UART without multithread strategy.

// ####################################################
// Include libraries:

#include <iostream>
#include <chrono>
#include <thread>
#include "../GpsSerialComLinux.h"

using namespace std;

// #####################################################
// Define parameters:

#define BAUDRATE                115200        // UART baudrate for GPS communication
#define SERIAL_PORT_ADDRESS     "/dev/ttyS0"

// #######################################################
// Define global variables and objects:

// GPS object 
GpsSerialComLinux gps;

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

    #if(GPIO_TYPE == 1)
        // Initial pigpio library.
        if (gpioInitialise() < 0) 
        {
            printf("pigpio initialization failed");
            return false;
        }
        gps.setPPSpin(GPS_PPS_PIN);
    #endif

    gps.setSerialPortAddress(SERIAL_PORT_ADDRESS);
    gps.setBaudrate(BAUDRATE);
    gps.parameters.TIME_OFFSET = 800;
    gps.parameters.PPS_PIN = -1;

    if(gps.begin())
    {
        printf("GPS object inited and setup finished.\n");
    }
    else
    {
        printf("%s\n",gps.errorMessage.c_str());
    }

    loop();

    // Close all resources of gps object.
    gps.clean();

    #if(GPIO_TYPE == 1)
        // Cleanup before exiting. This function is built in pigpio library
        gpioTerminate(); 
    #endif

    return 0;
}

// #######################################################
// Functions:

void loop(void)
{
    // gps.startThread();
    while(true)
    {
        t1 = micros();
        gps.update();
        
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

      
        //Wait for 1000 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
 
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