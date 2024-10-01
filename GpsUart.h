#ifndef GPS_UART_H
#define GPS_UART_H

// ######################################################################
// Information:
/*
    This library is for use GPS UART communication with raspberry pi. 
    It use TinyGPS++ library for GPS NMEA data parsing that written by mohammad nikanjam.
    Features of this library:
        1- UART communication and configuration for read/write GPS serial data.
        2- GPS NMEA data parsing by TinyGPS++ library. 
        3- Manage time by chrono library. eg for get micros() functions.
        4- Used pigpio library for set PPS GPIO pin and PPS signal interrupt. PPS (Pulse Per Second) pin of GPS trigged exact every 1 second.
        6- It can set any GPIO pin for PPS signal and interrupt.
        7- UART configuration is based on non blocking mode. So read/write communication work with no stop code.
        8- UART baude rate can set to custom value. 9600 or 115200. default value is 9600.
        9- PPS signal configuration is optionalal and it can work without PPS signal. Although with PPS signal, GPS time data will has more accuracy and reliability. 
        10- Multi thread option provided for GPS data update in real time applications if needed.   
        11- GPS data update method can be use optionally by seperate thread or just use it manually without seperated thread.
*/

// #############################################################
// Include libraries:

#include <iostream>
#include <fcntl.h>                                  // For file system management
#include <termios.h>                                // Use for UART communication
#include <fstream>                                  // For file system management
#include "../TinyGPSPlus/TinyGPSPlus.h"                 // For GPS data parsing.
#include <sys/ioctl.h>                              // Required for ioctl() function -> for get method  avilable char in uart recieved.
#include <unistd.h>                                 // it provides access to many system-level functions and constants necessary for various tasks like process management, file I/O, and system interaction.
#include <pigpio.h>                                 // For GPIO configuration            
#include <chrono>                                   // For time management
#include <thread>
#include <mutex>

// ###############################################################

// GPS_DATA structure for store gps data
struct GPS_DATA
{
    volatile uint64_t ppsCount;         // Auxilliary variable for GPS pulse per second counter. 
    float lat;                          // GPS Latitude. [deg] (-90, 90)
    float lon;                          // GPS Longitude. [deg] (0, 360)
    float alt;                          // GPS Altitude from seal level. [m]
    float course;                       // GPS motion course line relative to north.
    float speed;                        // GPS speed magnitude.
    double utcTime[6];                  // GPS UTC time: {Year, Month, Day, Hour, Minutes, Seconds}
    float hdop = 100;                   // GPS Hdop value.
    bool fixed;                         // GPS flag for 3d fixed mode.
    volatile bool ppsFlag;              // Auxilliary variable for pulse per second trigger.
    volatile uint64_t T_pps;            // Auxilliary variable for time at GPS PPS interrupt events. [us]
};

// RaspGPS_UART object
class RaspGPS_UART
{
    public:

        // GPS data structure.
        static GPS_DATA data;           

        // Store last error accured for object.
        static std::string errorMessage;       

        // Init and setup without thread configuration. setup uart port.
        static bool begin(void);

        // Start thread for update GPS data automatically.
        static bool startThread(void);

        // Stop thread for update GPS data automatically.
        static bool stopThread(void);

        // Set GPIO pin and enable interrupt handle function for GPS PPS signals.
        static void attachPPS(uint8_t pin);

        // Set GPIO pin for GPS PPS signals. 
        // Hint: after it need call begin() to enable action of pps interrupts.  
        static void setPPSpin(uint8_t pin);

        // Clean all setting for PPS signal. 
        static void detachPPS(void);

        // Set UART baudrate. default value is 115200.
        // @param rate: can be just 9600 or 115200.
        // @return true if successed.
        static bool setBaudrate(uint32_t rate);

        // Update GPS data. parse data available on uart and estimate utc time.
        static void update(void);

        // Clean and close all resources.
        static void clean(void);

    private:

        // The TinyGPSPlus object. This object get GPS stream data and parse it to states variables.
        // Hint: GPS data stream should read from UART port by another way before use TinyGPSPlus parser.
        static TinyGPSPlus _GPS;

        static double _utcTimeRef[6];              // UTC time at origin reference calculate in process. double [6] {Year, Month, Day, Hour, Minutes, Seconds}

        static int _serialPort;                    // Serial port number for open uart file. it is set automaticly in program.

        static char *_gpsStream;                   // Pointer for GPS UART data stream.

        static int8_t _ppsPin;               // GPIO pin number for GPS PPS pin interrupt.

        static uint32_t _baudRate;

        static std::thread _thread;
        static bool stopThreadFlag;

        // Time origin point. Reference time point for micros() calculations.
        static std::chrono::time_point<std::chrono::system_clock> _T_origin;

        // Return time from origin time point (T_origin: created at begin() method). [us]
        static uint64_t _micros(void);

        // Setup and init UART port.
        static bool _uartSetup(void);

        // Set utc reference when gps serial data recieved and valid.
        static void _setUtcReference(void);

        // Estimate high precicion utc time when call this function. calculate utc seconds with fractional section. 
        static void _calcUtcTime(void);

        // GPS functions for get GPS stream and convert it to gps data states.
        static void _parse(void);

        /*
            GPS PPS pulses external interrupts handle function.
            When GPS pulse per seconds trigged, this function execute.
        */ 
        static void _InterruptHandler(int /*gpio*/, int /*level*/, uint32_t /*tick*/); 
        
        // Thread function handler for call GPS update() continously in seperated thread until call stopThread().
        static void _updateThread(void);

};


#endif  // RASP_GPS_UART_H

