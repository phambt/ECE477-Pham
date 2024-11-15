#include <opencv2/opencv.hpp>
#include <SDL2/SDL.h>
#include <iostream>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <fcntl.h>      // For file controls like O_RDWR
#include <termios.h>    // For terminal controls like configuring UART
#include <unistd.h>     // For read/write
#include <cstring>      // For memset

int SIZE = 12;

// Struct for 12 bytes of data
typedef struct {
    uint8_t bytes[12];
} uint96_t;

// ** UART FUNCTIONS **
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

// ** Controller & Camera Code **

// Deadzone threshold for joystick drift
const int JOYSTICK_DEADZONE = 8000;

// Lookup table for button names
std::map<int, std::string> buttonLookup = {
    {SDL_CONTROLLER_BUTTON_A, "A"},
    {SDL_CONTROLLER_BUTTON_B, "B"},
    {SDL_CONTROLLER_BUTTON_X, "X"},
    {SDL_CONTROLLER_BUTTON_Y, "Y"},
    {SDL_CONTROLLER_BUTTON_BACK, "Back"},
    {SDL_CONTROLLER_BUTTON_GUIDE, "Guide"},
    {SDL_CONTROLLER_BUTTON_START, "Start"},
    {SDL_CONTROLLER_BUTTON_LEFTSTICK, "Left Stick"},
    {SDL_CONTROLLER_BUTTON_RIGHTSTICK, "Right Stick"},
    {SDL_CONTROLLER_BUTTON_LEFTSHOULDER, "Left Shoulder"},
    {SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, "Right Shoulder"},
    {SDL_CONTROLLER_BUTTON_DPAD_UP, "D-Pad Up"},
    {SDL_CONTROLLER_BUTTON_DPAD_DOWN, "D-Pad Down"},
    {SDL_CONTROLLER_BUTTON_DPAD_LEFT, "D-Pad Left"},
    {SDL_CONTROLLER_BUTTON_DPAD_RIGHT, "D-Pad Right"}
};

// Function to handle toggling based on bumper buttons
void handleToggle(SDL_GameController* controller, bool& toggleState, std::string& overlayText) {
    static bool leftBumperPressed = false;
    static bool rightBumperPressed = false;

    // Check the left bumper
    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) {
        if (!leftBumperPressed) { // Only toggle on the initial press
            toggleState = !toggleState;
            overlayText = toggleState ? "Mode: Fast" : "Mode: Slow";
            leftBumperPressed = true;
        }
    } else {
        leftBumperPressed = false;
    }

    // Check the right bumper
    if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) {
        if (!rightBumperPressed) { // Only toggle on the initial press
            toggleState = !toggleState;
            overlayText = toggleState ? "Mode: Fast" : "Mode: Slow";
            rightBumperPressed = true;
        }
    } else {
        rightBumperPressed = false;
    }
}

// Helper function to scale, apply sign, and convert to bytes
auto convertToMotorBytes = [](int value) -> std::array<uint8_t, 2> {
    std::array<uint8_t, 2> result;
    
    // Scale value
    int scaledValue = value / 6000;
    
    // Determine sign and apply it to the high nibble (top 4 bits)
    if (scaledValue >= 0) {
        result[0] = (scaledValue & 0x0F) | 0xF0; // Positive: top 4 bits 1111
    } else {
        result[0] = (scaledValue & 0x0F); // Negative: top 4 bits 0000
    }
    
    result[1] = static_cast<uint8_t>(scaledValue & 0xFF); // Lower 8 bits

    return result;
};

uint96_t updateUARTNum(int leftStickX, int leftStickY, int rightStickX, int rightStickY) {
    uint96_t uart_send;
    
    // Define motors as arrays to hold the two bytes for each motor
    std::array<uint8_t, 2> motor1, motor2, motor3, motor4, motor5, motor6;

    // Convert stick values to motor bytes
    motor1 = convertToMotorBytes(leftStickX);
    motor2 = convertToMotorBytes(leftStickY);
    motor3 = convertToMotorBytes(rightStickX);
    motor4 = convertToMotorBytes(rightStickY);
    motor5 = convertToMotorBytes(leftStickX + leftStickY); // Example additional motors
    motor6 = convertToMotorBytes(rightStickX + rightStickY);

    // Concatenate motor values into uart_send.bytes
    std::memcpy(&uart_send.bytes[0], motor1.data(), 2);
    std::memcpy(&uart_send.bytes[2], motor2.data(), 2);
    std::memcpy(&uart_send.bytes[4], motor3.data(), 2);
    std::memcpy(&uart_send.bytes[6], motor4.data(), 2);
    std::memcpy(&uart_send.bytes[8], motor5.data(), 2);
    std::memcpy(&uart_send.bytes[10], motor6.data(), 2);

    return uart_send;
}

// Time formatting for timer
std::string formatTime(int totalSeconds) {
    int minutes = totalSeconds / 60;
    int seconds = totalSeconds % 60;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << minutes << ":"
        << std::setw(2) << std::setfill('0') << seconds;
    return oss.str();
}

int main() {
    // ** UART **
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
    uint96_t uart_send = { .bytes = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
    bool sending = true;

    // ** Controller & Camera **
    // Initialize two cameras
    cv::VideoCapture cam1(0);
    cv::VideoCapture cam2(2);
    if (!cam1.isOpened() || !cam2.isOpened()) {
        std::cerr << "Error: Could not open one or both cameras." << std::endl;
        return -1;
    }
    cam1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cam1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cam2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cam2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Initialize SDL2 for game controller input
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    SDL_GameController* controller = SDL_GameControllerOpen(0);
    if (controller == nullptr) {
        std::cerr << "Could not open game controller! SDL Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    // Create a window to display the camera output
    cv::namedWindow("Camera Output", cv::WINDOW_AUTOSIZE);

    bool quit = false;
    bool timerRunning = false;
    bool toggleCamera = true; // Start with cam1
    bool toggleState = false;
    std::string overlayText = "Mode: Slow";
    auto startTime = std::chrono::steady_clock::now();
    int elapsedTime = 0;

    while (!quit) {
        // ** UART
        sendMessage(uart_fd, uart_send);
        receiveMessage(uart_fd);

        // ** Controller & Camera
        // Handle SDL events
        SDL_Event e;
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        // Toggle camera source with the "A" button
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A)) {
            toggleCamera = !toggleCamera;
            SDL_Delay(200); // Small delay to avoid rapid toggling
        }

        // Capture frame from the active camera
        cv::Mat frame;
        if (toggleCamera) {
            cam1 >> frame;
        } else {
            cam2 >> frame;
        }
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame." << std::endl;
            break;
        }

        // Handle controller bumper toggling for "Toggle 1" or "Toggle 0"
        handleToggle(controller, toggleState, overlayText);

        // Toggle timer with the 'Start' button
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_START)) {
            timerRunning = !timerRunning;
            if (timerRunning) {
                startTime = std::chrono::steady_clock::now();
            } else {
                elapsedTime += std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();
            }
        }

        // Handle Joystick input's
        int leftStickX = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
        int leftStickY = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
        int rightStickX = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX);
        int rightStickY = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY);

        // Apply deadzone filtering to prevent slight joystick drift
        if (abs(leftStickX) < JOYSTICK_DEADZONE) leftStickX = 0;
        if (abs(leftStickY) < JOYSTICK_DEADZONE) leftStickY = 0;
        if (abs(rightStickX) < JOYSTICK_DEADZONE) rightStickX = 0;
        if (abs(rightStickY) < JOYSTICK_DEADZONE) rightStickY = 0;

        if( leftStickX || leftStickY || rightStickX || rightStickY ){
            uart_send = updateUARTNum(leftStickX, leftStickY, rightStickX, rightStickY);
        }

        // Reset timer with the 'Back' button
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_BACK)) {
            timerRunning = false;
            elapsedTime = 0;
        }

        // Update displayed time if the timer is running
        int currentDisplayTime = elapsedTime;
        if (timerRunning) {
            currentDisplayTime += std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();
        }

        std::string timeText = "Time: " + formatTime(currentDisplayTime);
        int textX = 50, textY = 50;
        int textWidth = 300, textHeight = 80;

        // Draw the semi-transparent background rectangle
        cv::Mat overlay;
        frame.copyTo(overlay);
        cv::rectangle(overlay, cv::Point(textX - 10, textY - 40), cv::Point(textX + textWidth, textY + textHeight - 10), cv::Scalar(0, 0, 0), cv::FILLED);
        double alpha = 0.6;
        cv::addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame);

        // Display overlay text and formatted timer on the camera frame
        cv::putText(frame, overlayText, cv::Point(textX, textY), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, timeText, cv::Point(textX, textY + 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

        // Check if the X button is pressed to exit
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_X)) {
            quit = true;
        }

        // Show the frame
        cv::imshow("Camera Output", frame);

        // Wait a short time to reduce CPU load
        cv::waitKey(30);
    }

    // Clean up
    close(uart_fd); 

    cam1.release();
    cam2.release();
    SDL_GameControllerClose(controller);
    SDL_Quit();
    cv::destroyAllWindows();

    return 0;
}
