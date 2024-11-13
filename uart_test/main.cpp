#include <iostream>
#include <fcntl.h>      // For file controls like O_RDWR
#include <termios.h>    // For terminal controls like configuring UART
#include <unistd.h>     // For read/write
#include <cstring>      // For memset

// Function to configure UART for both Jetson and STM32
int configureUART(int uart_fd) {
    struct termios options;
    tcgetattr(uart_fd, &options); // Get current port attributes

    // Set baud rate to match STM32 settings
    cfsetispeed(&options, B9600); // Input baud rate
    cfsetospeed(&options, B9600); // Output baud rate

    options.c_cflag &= ~PARENB;    // No parity
    options.c_cflag &= ~CSTOPB;    // 1 stop bit
    options.c_cflag &= ~CSIZE;     
    options.c_cflag |= CS8;        // 8 bits per byte
    options.c_cflag &= ~CRTSCTS;   // No hardware flow control (can enable if STM32 supports it)
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    options.c_lflag &= ~ICANON;    // Disable canonical mode
    options.c_lflag &= ~ECHO;      // Disable echo
    options.c_lflag &= ~ECHOE;     // Disable erasure
    options.c_lflag &= ~ECHONL;    // Disable new-line echo
    options.c_lflag &= ~ISIG;      // Disable interpretation of INTR, QUIT and SUSP
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
    options.c_iflag &= ~(ICRNL | INLCR); // Disable CR to NL translation, NL to CR translation
    options.c_oflag &= ~OPOST; // Disable output processing

    options.c_cc[VMIN]  = 1;      // Read at least 1 character (blocking)
    options.c_cc[VTIME] = 0;      // No timeout (blocking)

    tcflush(uart_fd, TCIFLUSH);   // Flush old input
    return tcsetattr(uart_fd, TCSANOW, &options); // Apply attributes
}

// Function to send a 16-bit number (two bytes) to STM32
void sendMessage(int uart_fd, uint16_t number) {
    uint8_t buffer[2];
    buffer[0] = number >> 8;    // High byte
    buffer[1] = number & 0xFF;  // Low byte

    // Clear the UART output buffer to avoid data clogs
    tcflush(uart_fd, TCOFLUSH);

    // Send both bytes together as a single buffer
    ssize_t bytes_written = write(uart_fd, buffer, 2);
    if (bytes_written != 2) {
        std::cerr << "Failed to write both bytes to UART" << std::endl;
    } else {
        std::cout << "Sending number: " << number << " (0x" << std::hex << number << std::dec << ")" << std::endl;
    }
    usleep(100000); // Delay of 100ms after each send attempt
}



// Function to receive a 16-bit number (two bytes) from STM32
bool receiveMessage(uint16_t uart_fd) {
    uint8_t buffer[2];
    ssize_t bytes_read = read(uart_fd, buffer, 2);

    if (bytes_read == 2) {
        uint16_t received_number = (buffer[0] << 8) | buffer[1];
        std::cout << "\nMessage received from STM32: " << received_number << std::endl;
        return true; // Data was received
    } else {
        std::cerr << "Failed to read from UART or insufficient data received" << std::endl;
        return false; // No data was received
    }
}

// Function to check if there is data available to read from STM32
bool isDataAvailable(int uart_fd) {
    fd_set read_fds;
    struct timeval timeout;

    // Clear the set and add the UART file descriptor to the set
    FD_ZERO(&read_fds);
    FD_SET(uart_fd, &read_fds);

    // Set timeout to zero, this makes select non-blocking
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    // Check if data is available to read
    int result = select(uart_fd + 1, &read_fds, NULL, NULL, &timeout);

    return (result > 0 && FD_ISSET(uart_fd, &read_fds));
}

int main() {
    // Open UART device in blocking mode (without O_NDELAY)
    int uart_fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY);
    if (uart_fd == -1) {
        std::cerr << "Failed to open /dev/ttyTHS0" << std::endl;
        return -1;
    }

    // Configure the UART for communication with STM32
    if (configureUART(uart_fd) != 0) {
        std::cerr << "Failed to configure UART" << std::endl;
        close(uart_fd);
        return -1;
    }

    // Example 16-bit number to send to STM32
    uint16_t number = 61367;
    bool sending = true;

    bool first_send = true;

    // Main loop: Wait for a response before sending the next message
    while (true) { 
        // if( first_send ){
        //     sendMessage(uart_fd, number);

        //     if (isDataAvailable(uart_fd) && receiveMessage(uart_fd)) {
        //         std::cout << " X " << std::endl;
        //         sending = false;
        //         first_send = false;
        //     }
        // }
        
        if (!sending) {
            if (receiveMessage(uart_fd)) {
                sending = true;
            }
        } else {
            sendMessage(uart_fd, number);

            if (isDataAvailable(uart_fd) && receiveMessage(uart_fd)) {
                sending = false;
                first_send = false;
            }
        }
    }

    // Close UART
    close(uart_fd);
    return 0;
}
