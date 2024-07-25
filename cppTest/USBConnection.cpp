#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

int main() {
    std::cout << "hello" << std::endl;
    // Open the serial port
    int serial_port = open("/dev/ttyUSB1", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        std::cerr << "Error " << errno << " opening " << "/dev/ttyUSB1" << ": " << strerror(errno) << std::endl;
        return 1;
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return 1;
    }

    // Set Baud Rate
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // Setting other Port Stuff
    tty.c_cflag &= ~PARENB; // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS; // no flow control
    tty.c_cc[VMIN] = 1; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

    // Make raw
    cfmakeraw(&tty);

    // Flush Port, then applies attributes
    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr" << std::endl;
        return 1;
    }

    // Write to serial port
    const char *msg = "G";
    write(serial_port, msg, sizeof(msg));
    std::cout << "Sent Message \"G\" to serial" << std::endl;

    // Read from serial port
    char read_buf [256];
    memset(&read_buf, '\0', sizeof(read_buf));
    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
    if (num_bytes < 0) {
        std::cerr << "Error reading: " << strerror(errno) << std::endl;
        return 1;
    }

    // Here we assume we received a string, so we null-terminate it
    read_buf[num_bytes] = '\0';

    std::cout << "Read " << num_bytes << " bytes. Received message: " << read_buf << std::endl;

    // Close the serial port
    close(serial_port);

    return 0;
}