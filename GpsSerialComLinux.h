#pragma once

/**
 * @brief Define for GPIO type selection.  
 * 
 * 0: General PC systems
 * 
 * 1: Raspberry pi boards
 */
#define GPIO_TYPE   0

// ##############################################################
// Include libraries:

#include <iostream>
#include <fcntl.h>                                      // For file system management
#include <termios.h>                                    // Use for UART communication
#include <fstream>                                      // For file system management
#include "./libraries/TinyGPSPlus_OS/TinyGPSPlus.h"     // For GPS data parsing.
#include <sys/ioctl.h>                                  // Required for ioctl() function -> for get method  avilable char in uart recieved.
#include <unistd.h>                                     // it provides access to many system-level functions and constants necessary for various tasks like process management, file I/O, and system interaction.  
#include <chrono>                                       // For time management
#include <thread>
#include <mutex>

#if(GPIO_TYPE == 1)
    #include <pigpio.h>                                 // For GPIO access and configuration    
#endif 

#if(GPIO_TYPE == 0)
    #include <gpiod.h>                                  // For GPIO access and configuration 
#endif

// ###############################################################
// Public Structures:

/**
 * @struct GPS_DATA
 * @brief GPS_DATA structure for store gps data
 *  */ 
struct GPS_DATA
{
    volatile uint64_t ppsCount = 0 ;        // Auxilliary variable for GPS pulse per second counter. 
    float lat = 0 ;                         // GPS Latitude. [deg] (-90, 90)
    float lon = 0;                          // GPS Longitude. [deg] (-360, 360)
    float alt = 0;                          // GPS Altitude from seal level. [m]
    float course = 0;                       // GPS motion course line relative to north.
    float speed = 0;                        // GPS speed magnitude.
    double utcTime[6];                      // GPS UTC time: {Year, Month, Day, Hour, Minutes, Seconds}
    float hdop = 100;                       // GPS Hdop value.
    bool fixed = false;                     // GPS flag for 3d fixed mode.
    volatile bool ppsFlag = false;          // Auxilliary variable for pulse per second trigger.
    volatile uint64_t T_pps = 0;            // Auxilliary variable for time at GPS PPS interrupt events. [us]
};

// #######################################################################################
// Public classes:

/**
 * @class GpsSerialComLinux
 * @brief A class for UART or RS232 communication with GPS. 
 * @note This class parses raw NMEA data from GPS.
 *  */  
class GpsSerialComLinux
{
    public:

        /// @brief GPS data structure.
        static GPS_DATA data;           

        /// @brief Last error occurred for the object.
        static std::string errorMessage;   

        /**
         * @struct ParametersStructure
         * @brief Parameters structure.
         *  */    
        static struct ParametersStructure
        {
            /**
             * @brief Serial port number for open uart file. it is set automaticly in program.
             * @note An example value: "/dev/ttyS0"
             */
            static std::string SERIAL_PORT_ADDRESS;

            /**
             * @brief GPIO pin number for GPS PPS pin interrupt.
             * @note - A value of -1 means it is disabled.
             * @note - Any change of it need call begin() to enable action of pps interrupts.
             */
            static int8_t PPS_PIN;           

            /**
             * @brief Baud rate for serial communication.
             * @note A value of either 9600 or 115200 is acceptable.
             */
            static uint32_t BAUDRATE;

            /**
             * @brief GPS time data offset. [us]
             */
            static uint32_t TIME_OFFSET;
        }parameters;

        /**
         * @brief Init and setup without thread configuration. setup uart port.
         * @return true is succeeded.
         *  */ 
        static bool begin(void);

        /**
         * @brief Start thread for update GPS data automatically.
         * @return true if succeeded.
         *  */ 
        static bool startThread(void);

        /**
         * @brief Stop thread for update GPS data automatically.
         * @return true if succeeded.
         *  */ 
        static bool stopThread(void);

        /**
         * @brief Set GPIO pin and enable interrupt handle function for GPS PPS signals.
         *  */ 
        static void attachPPS(uint8_t pin);

        // // Set GPIO pin for GPS PPS signals. 
        // // Hint: after it need call begin() to enable action of pps interrupts.  
        // static void setPPSpin(uint8_t pin);

        // Clean all setting for PPS signal. 
        static void detachPPS(void);

        /**
         * @brief Set UART baudrate. default value is 9600.
         * @param rate: can be just 9600 or 115200.
         * @return true if successed.
         *  */ 
        static bool setBaudrate(uint32_t rate);

        static void setSerialPortAddress(std::string address);

        /**
         * @brief Update GPS data. parse data available on uart and estimate utc time.
         * @param nonBlockEnable: Is flag for setup serial communication at blocking mode or non blocking mode.
         * @note Default value of nonBlockEnable is false.
         *  */ 
        static void update(bool nonBlockEnable = false);

        /// @brief Clean and close all resources.
        static void clean(void);

    private:

        static bool _isPortOpenedFlag;

        static uint64_t _T_SerialTimeUpdate;

        static std::tm _timeInfo;

        /**
         * @brief The TinyGPSPlus object. This object get GPS stream data and parse it to states variables.
         * @note GPS data stream should read from UART port by another way before use TinyGPSPlus parser.
         *  */ 
        static TinyGPSPlus _GPS;

        /**
         * @brief UTC time at origin reference calculate in process. double [6] {Year, Month, Day, Hour, Minutes, Seconds}
         *  */ 
        static double _utcTimeRef[6];              

        /**
         * @brief Serial port number for open uart file. it is set automaticly in program.
         *  */ 
        static int _serialPort;                    

        /**
         * @brief Pointer for GPS UART data stream.
         *  */ 
        static char *_gpsStream;                   

        static std::thread _thread;

        static bool stopThreadFlag;

        /**
         * @brief Time origin point. Reference time point for micros() calculations.
         *  */ 
        static std::chrono::time_point<std::chrono::system_clock> _T_origin;

        /**
         * @brief Return time from origin time point (T_origin: created at begin() method). [us]
         *  */ 
        static uint64_t _micros(void);

        /**
         * @brief Setup and init UART port.
         * @param nonBlockEnable: Is flag for setup serial communication at blocking mode or non blocking mode.
         * @note Default value of nonBlockEnable is false.
         *  */ 
        static bool _uartSetup(bool nonBlockEnable = false);

        /**
         * @brief Set utc reference when gps serial data recieved and valid.
         *  */ 
        static void _setUtcReference(void);

        /**
         * @brief Estimate high precicion utc time when call this function. calculate utc seconds with fractional section. 
         *  */ 
        static void _calcUtcTime(void);

        /**
         * @brief GPS functions for get GPS stream and convert it to gps data states.
         * @param nonBlockEnable: Is flag for setup serial communication at blocking mode or non blocking mode.
         * @note Default value of nonBlockEnable is false
         *  */ 
        static void _parse(bool nonBlockEnable = false);

        /**
         * @brief GPS PPS pulses external interrupts handle function.
         * When GPS pulse per seconds trigged, this function execute.
         */
        static void _InterruptHandler(int /*gpio*/, int /*level*/, uint32_t /*tick*/); 
        
        /**
         * @brief Thread function handler for call GPS update() continously in seperated thread until call stopThread().
         *  */ 
        static void _updateThread(void);

        /**
         * @brief Check parameters validation.
         * @return true if succeeded.
         */
        static bool _checkParameters(void);
};






