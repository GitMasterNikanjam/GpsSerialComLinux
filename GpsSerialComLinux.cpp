#include "GpsSerialComLinux.h"

// ####################################################################

// Define the static member variable: data
GPS_DATA RaspGPS_UART::data;
std::chrono::time_point<std::chrono::system_clock> RaspGPS_UART::_T_origin;
std::string RaspGPS_UART::errorMessage;
TinyGPSPlus RaspGPS_UART::_GPS;
double RaspGPS_UART::_utcTimeRef[6];
int RaspGPS_UART::_serialPort = -1; // Default initialization
char* RaspGPS_UART::_gpsStream = nullptr;
int8_t RaspGPS_UART::_ppsPin = -1;
uint32_t RaspGPS_UART::_baudRate = 115200;
std::thread RaspGPS_UART::_thread;
bool RaspGPS_UART::stopThreadFlag = false;

// ########################################################################

bool RaspGPS_UART::_uartSetup(void)
{
    // Open UART device file in read-write mode, prevent it from becoming the controlling terminal, and set non-blocking mode
    _serialPort = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_serialPort == -1) 
    {
        errorMessage = "Error opening UART device file.";
        return false;
    }

    // Configure UART settings
    struct termios uart_config;
    tcgetattr(_serialPort, &uart_config);
    
    uint baudRate;
    switch (_baudRate)
    {
    case 9600:
        baudRate = B9600;
        break;
    case 115200:
        baudRate = B115200;
        break;
    default:
        baudRate = B115200;
        break;
    }

    cfsetispeed(&uart_config, baudRate); // Set baud rate to 115200
    cfsetospeed(&uart_config, baudRate);
    uart_config.c_cflag &= ~PARENB; // No parity
    uart_config.c_cflag &= ~CSTOPB; // 1 stop bit
    uart_config.c_cflag &= ~CSIZE;  // 8 data bits
    uart_config.c_cflag |= CS8;
    tcsetattr(_serialPort, TCSANOW, &uart_config);

    // Clear the input buffer
    tcflush(_serialPort, TCIFLUSH);

    return true;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::_parse(void)
{
    // Open serialport for uart in non blocking mode.
    _serialPort = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);

    // Check number of characters available in UART receive buffer
    int uart_available_chars;
    ioctl(_serialPort, FIONREAD, &uart_available_chars);
    if (uart_available_chars > 0) 
    {
        char uart_read_buffer[uart_available_chars];
        int uart_bytes_read = read(_serialPort, &uart_read_buffer, sizeof(uart_read_buffer));
        if (uart_bytes_read == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) 
        {
            // Manage program for no data recieved from uart port.
        } 
        else if (uart_bytes_read > 0) 
        {
            // Data read successfully
            uart_read_buffer[uart_bytes_read] = '\0'; // Null-terminate the string
            _gpsStream = uart_read_buffer;
            while (*_gpsStream)

                if(_GPS.encode(*_gpsStream++))
                {
                    // Update gps location.
                    if(_GPS.location.isUpdated())
                    {
                        data.lat = _GPS.location.lat();
                        data.lon = _GPS.location.lng();
                        data.alt = _GPS.altitude.meters();
                    }

                    // Update gps speed.
                    if(_GPS.speed.isUpdated())
                    {
                        data.speed = _GPS.speed.mps();
                    }

                    // Update gps course.
                    if(_GPS.course.isUpdated())
                    {
                        data.course = _GPS.course.deg();
                    }

                    if(_GPS.time.isUpdated())
                    {
                        if(_ppsPin >= 0)
                        {
                            // Update utc time reference.
                            if((data.ppsFlag == true) && (_GPS.time.isValid()))
                            {
                                data.ppsCount = 0;
                                _setUtcReference();
                            }   
                        }
                        else
                        {
                            data.T_pps = _micros();
                            _setUtcReference();
                        }
                    }

                    // Update gps hdop value.
                    if(_GPS.hdop.isUpdated())
                    {
                        data.hdop = _GPS.hdop.hdop();
                    }

                    // Update gps fixed mode.
                    if(_GPS.location.isValid() && (_GPS.hdop.hdop() <=1.5) && (_GPS.satellites.value() >= 5))
                    {
                        data.fixed = true;
                    }
                    else
                    {
                        data.fixed = false;
                    }
                }
        }
    }

    // Close UART
    close(_serialPort);
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::_setUtcReference(void)
{  
    // Save GPS data and time to UTC_time_reference. 
    _utcTimeRef[0] = _GPS.date.year();
    _utcTimeRef[1] = _GPS.date.month();
    _utcTimeRef[2] = _GPS.date.day();
    _utcTimeRef[3] = _GPS.time.hour();
    _utcTimeRef[4] = _GPS.time.minute();
    _utcTimeRef[5] = _GPS.time.second();
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::_calcUtcTime(void)
{
    uint64_t T = _micros();            // current time. [us]
    uint64_t dT = T - data.T_pps;      // Deference between current time and last time in PPS interrupt. [us]

    // Check if pps pulses duration is longer than 1.2 seconds, then set ppsFlag to false.
    if((dT >= 1200000))
    {
        data.ppsFlag = false;
    }
        
    double dt[2];                                   // -> dt = {dt[0] = fractional section, dt[1] = integer section}
    dt[0] = int(dT/1000000.0) + data.ppsCount;      // Get integer section of time.
    dt[1] = double(dT%1000000)/1000000.0;           // Get fractional section of time.

    std::tm timeInfo;
    timeInfo.tm_year = _utcTimeRef[0] - 1900;       // years since 1900
    timeInfo.tm_mon = _utcTimeRef[1] - 1;           // months since January (0-based)
    timeInfo.tm_mday = _utcTimeRef[2];              // day of the month (1-31)
    timeInfo.tm_hour = _utcTimeRef[3];              // hours since midnight (0-23)
    timeInfo.tm_min = _utcTimeRef[4];               // minutes after the hour (0-59)
    timeInfo.tm_sec = _utcTimeRef[5];               // seconds

    // Convert std::tm to std::time_t
    std::time_t time = std::mktime(&timeInfo); 
    
    // Add integer section to UTC time seconds.
    time += dt[0];

    // Convert std::time_t back to std::tm
    // Hint it is used localtime not gmtime, beacuse time variable itself is in UTC time.
    timeInfo = *(std::localtime(&time));

    // Add fractional section of time.
    data.utcTime[0] = timeInfo.tm_year + 1900;
    data.utcTime[1] = timeInfo.tm_mon + 1;
    data.utcTime[2] = timeInfo.tm_mday;
    data.utcTime[3] = timeInfo.tm_hour;
    data.utcTime[4] = timeInfo.tm_min;
    data.utcTime[5] = timeInfo.tm_sec + dt[1];
     
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::_InterruptHandler(int /*gpio*/, int /*level*/, uint32_t /*tick*/) 
{
    uint64_t t = _micros();
    uint64_t dT = t - data.T_pps;
    if(dT >= 1500000)
    {
        data.ppsCount = int(dT/1000000.0) + data.ppsCount;      // Add integer section of time.
    }
    data.T_pps = t;
    data.ppsFlag = true;
    data.ppsCount++;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RaspGPS_UART::begin(void)
{
    // Save time point for time origin.
    _T_origin = std::chrono::high_resolution_clock::now();
    
    if(!_uartSetup())
    {
        return false;
    }

    if(_ppsPin >= 0)
    {
        attachPPS(_ppsPin);
    }

    return true;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RaspGPS_UART::startThread(void)
{
    // Create a thread to read temperature
    _thread = std::thread(_updateThread);

    return true;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RaspGPS_UART::stopThread(void)
{
    stopThreadFlag = true;
    if (_thread.joinable())
    {
        _thread.join(); // Wait for the thread to finish
    }

    return true;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint64_t RaspGPS_UART::_micros(void)
{
    std::chrono::time_point<std::chrono::system_clock> T_point;
    // Get Current time point
    T_point = std::chrono::high_resolution_clock::now();
    // Calculate Time duration
    auto T_dur = std::chrono::duration_cast<std::chrono::microseconds>(T_point - _T_origin);
    // Return time duration in uint64_t 
    return static_cast<uint64_t>(T_dur.count());
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::clean(void)
{
    stopThreadFlag = true;
    if (_thread.joinable())
    {
        _thread.join(); // Wait for the thread to finish
    }

    detachPPS();
    close(_serialPort);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::update(void)
{
    _parse();
    _calcUtcTime();
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::_updateThread(void)
{
    while(!stopThreadFlag)
    {
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Adjust the sleep duration as needed
    }
}
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::detachPPS(void)
{
    if(_ppsPin >= 0)
    {
        #if(GPIO_TYPE == 1)
            gpioSetMode(_ppsPin, PI_INPUT);
            gpioSetPullUpDown(_ppsPin, PI_PUD_OFF);
            gpioSetISRFunc(_ppsPin, RISING_EDGE,0, nullptr);           // Free interrupt handler for GPS PPS pin.
        #endif
    }

    _ppsPin = -1;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::attachPPS(uint8_t pin)
{
    _ppsPin = pin;

    #if(GPIO_TYPE == 1)
        gpioSetMode(_ppsPin, PI_INPUT);
        gpioSetPullUpDown(_ppsPin, PI_PUD_DOWN);
        gpioSetISRFunc(_ppsPin, RISING_EDGE,0, _InterruptHandler);  // Set up interrupt handler for GPS PPS pin.
    #endif
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RaspGPS_UART::setPPSpin(uint8_t pin)
{
    _ppsPin = pin;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool RaspGPS_UART::setBaudrate(uint32_t rate)
{
    if( (rate == 9600) || (rate == 115200) )
    {
        _baudRate = rate;
    }
    else
    {
        errorMessage = "Baudrate is not valid value.";
        return false;
    }
    
    return true;
}