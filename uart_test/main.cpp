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

// Function to send data to STM32
void sendMessage(int uart_fd, const char* message) {
    size_t len = strlen(message);
    ssize_t bytes_written = write(uart_fd, message, len);
    if (bytes_written != len) {
        std::cerr << "Failed to write to UART" << std::endl;
    } else {
        std::cout << "Message sent: " << message;
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
bool receiveMessage(int uart_fd) {
    uint8_t buffer[256];  // Using uint8_t buffer instead of char
    memset(buffer, 0, sizeof(buffer));
    ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);
    
    if (bytes_read > 0) {
        // Print each byte as character to ensure the data is printed correctly
        std::cout << "Message received from STM32: ";
        for (ssize_t i = 0; i < bytes_read; ++i) {
            std::cout << static_cast<char>(buffer[i]);
        }
        std::cout << std::endl;
        return true; // Data was received
    } else {
        std::cerr << "Failed to read from UART or no data received" << std::endl;
        return false; // No data was received
    }
}

// Function to receive data from STM32
// bool receiveMessage(int uart_fd) {
//     char buffer[256];
//     memset(buffer, 0, sizeof(buffer));
//     ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);
//     if (bytes_read > 0) {
//         std::cout << "Message received from STM32: " << buffer << std::endl;
//         return true; // Data was received
//     } else {
//         std::cerr << "Failed to read from UART or no data received" << std::endl;
//         return false; // No data was received
//     }
// }

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

    // Example message to send to STM32
    const char* message = "Jetson says hello!\n";
    // sendMessage(uart_fd, message);

    // Main loop: Wait for a response before sending the next message
    while (true) {
        // Check if data is available before sending another message
        // if (isDataAvailable(uart_fd)) {
        if (receiveMessage(uart_fd)) {
            // Only send the next message after receiving a response
            // sendMessage(uart_fd, message);
        }
        // }
    }

    // Close UART
    close(uart_fd);
    return 0;
}
