#include <iostream>
#include <fcntl.h>      // For file controls like O_RDWR
#include <termios.h>    // For terminal controls like configuring UART
#include <unistd.h>     // For read/write
#include <cstring>      // For memset

int SIZE = 12;

// Struct for 12 bytes of data
typedef struct {
    uint8_t bytes[12];
} uint96_t;

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
void sendMessage(int uart_fd, uint96_t number) {
    uint8_t buffer[SIZE];

    // Copy each byte from the struct into the buffer
    for (int i = 0; i < SIZE; i++) {
        buffer[i] = number.bytes[i]; // Extract each byte
    }

    // Clear the UART output buffer to avoid data clogs
    tcflush(uart_fd, TCOFLUSH);

    // Send all 8 bytes together as a single buffer
    ssize_t bytes_written = write(uart_fd, buffer, SIZE);
    if (bytes_written != SIZE) {
        std::cerr << "Failed to write 8 bytes to UART" << std::endl;
    } 
    else {
        std::cout << "Sending number: "; // Print out the num were sending (debugging)
        for( int i = 0; i < SIZE; i++ ){
            std::cout << " " << std::hex << static_cast<int>(buffer[i]) << std::dec;
        }
        std::cout << std::endl;
    }

    // usleep(100000); // Delay of 100ms after each send attempt
}



// Function to receive a 64-bit number (8 bytes) from STM32
bool receiveMessage(int uart_fd) {
    uint8_t buffer[SIZE];
    ssize_t bytes_read = read(uart_fd, buffer, SIZE);

    if (bytes_read == SIZE) {
        // Extract the number from the first two bytes
        // uint16_t received_number = (buffer[0] << 8) | buffer[1];
        // std::cout << "\nMessage received from STM32: " << received_number << std::endl;

        // Print the rest of the bytes for debugging (optional)
        std::cout << "Received bytes:";
        for (int i = 0; i < SIZE; i++) {
            std::cout << " " << std::hex << static_cast<int>(buffer[i]) << std::dec;
        }
        std::cout << std::endl;

        return true; // Data was received
    } else {
        std::cerr << "Failed to read 8 bytes from UART or insufficient data received" << std::endl;
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
    uint96_t number = { .bytes = { 0x11, 0x22, 0x43, 0x34, 0x69, 0xAB, 0xDE, 0xAD, 0xBE, 0xEF, 0x0B, 0xC0 } };
    bool sending = true;

    // Main loop: Wait for a response before sending the next message
    while (true) { 

        sendMessage(uart_fd, number);
        receiveMessage(uart_fd);

    }

    // Close UART
    close(uart_fd);
    return 0;
}
