#include <iostream>
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>      // C string library
#include <errno.h>      // For error handling

int main() {
    const char* port_name = "/dev/ttyS0"; // Replace with your GPS serial port
    int baud_rate = B9600;               // Common GPS baud rate (9600 bps)

    // Open the serial port (without O_NDELAY)
    int serial_port = open(port_name, O_RDWR | O_NOCTTY);
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

    std::cout << "Reading GPS data from " << port_name << "..." << std::endl;

    // Read and print data in a loop
    while (true) {
        memset(buffer, 0, sizeof(buffer)); // Clear the buffer
        int bytes_read = read(serial_port, buffer, sizeof(buffer) - 1);
        if (bytes_read < 0) {
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
        } else if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // Null-terminate the received data
            std::cout << "GPS Data: " << buffer << std::endl;
        }

        // Sleep briefly to avoid busy loop
        // usleep(100000); // 100ms delay
    }

    // Close the serial port
    close(serial_port);
    return 0;
}
