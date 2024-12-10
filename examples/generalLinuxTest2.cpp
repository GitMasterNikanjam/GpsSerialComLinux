#include <iostream>
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>      // C string library
#include <errno.h>      // For error handling
#include <chrono>
#include <thread>
#include <sys/select.h> // Include for select()

int main() {
    const char* port_name = "/dev/ttyS0"; // Replace with your GPS serial port
    int baud_rate = B9600;               // Common GPS baud rate (9600 bps)

    // Open the serial port in non-blocking mode
    int serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return 1;
    }

    // Configure the serial port
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting serial port attributes: " << strerror(errno) << std::endl;
        close(serial_port);
        return 1;
    }

    // Set baud rate
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;  // Clear the data size bits
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines

    tty.c_iflag = 0;         // Disable software flow control
    tty.c_lflag = 0;         // Disable canonical mode
    tty.c_oflag = 0;         // No special output processing

    // Apply the settings
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes: " << strerror(errno) << std::endl;
        close(serial_port);
        return 1;
    }

    // Buffer to store incoming data
    char buffer[256];

    std::cout << "Waiting for GPS data from " << port_name << "..." << std::endl;

    while (true) {
        int serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
        // Set up select() to wait for data with a timeout
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(serial_port, &read_fds);

        // Set timeout (in seconds)
        struct timeval timeout;
        timeout.tv_sec = 1; // 1 second timeout
        timeout.tv_usec = 0;

        int select_result = select(serial_port + 1, &read_fds, nullptr, nullptr, &timeout);

        if (select_result < 0) {
            std::cerr << "Select error: " << strerror(errno) << std::endl;
            break;
        } else if (select_result == 0) {
            std::cout << "[DEBUG] No data within timeout." << std::endl;
        } else if (FD_ISSET(serial_port, &read_fds)) {
            // Data is ready to read
            int bytes_read = read(serial_port, buffer, sizeof(buffer) - 1);

            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::cout << "GPS Data: " << buffer << std::endl;
            } else if (bytes_read < 0) {
                if (errno != EAGAIN) {
                    // Only log unexpected read errors
                    std::cerr << "Unexpected read error: " << strerror(errno) << std::endl;
                }
            }
        }

        close(serial_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Optional delay to prevent 100% CPU usage
    }

    close(serial_port);
    return 0;
}
