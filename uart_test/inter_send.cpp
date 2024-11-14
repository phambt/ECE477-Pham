#include <iostream>
#include <fcntl.h>      // For file controls like O_RDWR
#include <termios.h>    // For terminal controls like configuring UART
#include <unistd.h>     // For read/write
#include <cstring>      // For memset
#include <csignal>      // For signal handling

// Global variable for UART file descriptor
int uart_fd;

// Function to configure UART for Jetson
int configureUART(int uart_fd) {
    struct termios options;
    tcgetattr(uart_fd, &options); // Get current port attributes

    // Set baud rate to match STM32 settings
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= PARENB;      // Enable even parity
    options.c_cflag &= ~PARODD;     // Set to even parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8 bits per byte
    options.c_cflag &= ~CRTSCTS;    // No hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    options.c_lflag &= ~ICANON;    // Disable canonical mode
    options.c_lflag &= ~ECHO;      // Disable echo
    options.c_lflag &= ~ISIG;      // Disable interpretation of INTR, QUIT, and SUSP
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    options.c_oflag &= ~OPOST;     // Disable output processing

    options.c_cc[VMIN]  = 1;       // Read at least 1 character
    options.c_cc[VTIME] = 0;       // No timeout

    tcflush(uart_fd, TCIFLUSH);    // Flush old input
    fcntl(uart_fd, F_SETFL, O_ASYNC | O_NONBLOCK); // Set non-blocking and async
    fcntl(uart_fd, F_SETOWN, getpid());            // Set owner to receive SIGIO

    return tcsetattr(uart_fd, TCSANOW, &options); // Apply attributes
}

// Signal handler for UART input on Jetson
void uartSignalHandler(int signo) {
    uint8_t buffer[4];
    ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer));
    if (bytes_read == sizeof(buffer)) {
        std::cout << "Received bytes:";
        for (int i = 0; i < 4; ++i) {
            std::cout << " 0x" << std::hex << static_cast<int>(buffer[i]);
        }
        std::cout << std::dec << std::endl;

        if (buffer[0] == 0x7E && buffer[3] == 0x7F) {  // Check framing
            uint16_t received_number = (buffer[1] << 8) | buffer[2];
            std::cout << "Received from STM32: " << received_number << std::endl;
        } else {
            std::cerr << "Frame error: Invalid start/end bytes" << std::endl;
        }
    } else {
        std::cerr << "Incomplete message. Bytes read: " << bytes_read << std::endl;
    }
}

// Function to send a framed message to STM32
void sendMessage(uint16_t number) {
    uint8_t buffer[4] = {0x7E, static_cast<uint8_t>(number >> 8), static_cast<uint8_t>(number & 0xFF), 0x7F};
    write(uart_fd, buffer, sizeof(buffer));
}

int main() {
    uart_fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY);
    if (uart_fd == -1) {
        std::cerr << "Failed to open UART" << std::endl;
        return -1;
    }

    if (configureUART(uart_fd) != 0) {
        std::cerr << "Failed to configure UART" << std::endl;
        close(uart_fd);
        return -1;
    }

    // Set up signal handler for SIGIO
    struct sigaction saio;
    saio.sa_handler = uartSignalHandler;
    sigemptyset(&saio.sa_mask);
    saio.sa_flags = 0;
    sigaction(SIGIO, &saio, NULL);

    uint16_t number = 1;  // Example number to send

    // Main loop: Continuously send data to STM32 and wait for response via interrupt
    while (true) {
        sendMessage(number);
        number++;  // Increment the number each time for testing
        sleep(1);  // Send every second
    }

    close(uart_fd);
    return 0;
}
