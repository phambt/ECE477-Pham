#include <opencv2/opencv.hpp>
#include <SDL2/SDL.h>
#include <iostream>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>

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

std::string formatTime(int totalSeconds) {
    int minutes = totalSeconds / 60;
    int seconds = totalSeconds % 60;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << minutes << ":"
        << std::setw(2) << std::setfill('0') << seconds;
    return oss.str();
}

int main() {
    // Initialize OpenCV camera
    cv::VideoCapture cap(1); 
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

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

    // Load the overlay image
    cv::Mat overlayImage = cv::imread("NICE.png");
    if (overlayImage.empty()) {
        std::cerr << "Error: Could not load overlay image." << std::endl;
        return -1;
    }
    cv::resize(overlayImage, overlayImage, cv::Size(640, 480)); // Resize to match camera frame size

    // Create a window to display the camera output
    cv::namedWindow("Camera Output", cv::WINDOW_AUTOSIZE);

    // Main loop flag, overlay text, timer, and toggle variables
    bool quit = false;
    bool timerRunning = false;
    bool showCameraFeed = true; // Flag to toggle between camera feed and overlay image
    bool toggleState = false;   // Initial toggle state
    std::string overlayText = "Mode: Slow"; // Default overlay text
    auto startTime = std::chrono::steady_clock::now();
    int elapsedTime = 0; // in seconds

    while (!quit) {
        // Handle SDL events
        SDL_Event e;
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        // Check if the "A" button is pressed to toggle display
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A)) {
            showCameraFeed = !showCameraFeed; // Toggle the display
            SDL_Delay(200); // Small delay to avoid rapid toggling
        }

        // Capture frame from the camera if the camera feed is active
        cv::Mat frame;
        if (showCameraFeed) {
            cap >> frame;
            if (frame.empty()) {
                std::cerr << "Error: Could not capture frame." << std::endl;
                break;
            }
        } else {
            // Use the overlay image instead of the camera frame
            frame = overlayImage.clone();
        }

        // Handle controller bumper toggling for "Toggle 1" or "Toggle 0"
        handleToggle(controller, toggleState, overlayText);

        // Toggle timer with the 'Start' button
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_START)) {
            timerRunning = !timerRunning;
            if (timerRunning) {
                startTime = std::chrono::steady_clock::now(); // Reset start time
            } else {
                elapsedTime += std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();
            }
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

        // Set up positions for the text and background rectangle
        std::string timeText = "Time: " + formatTime(currentDisplayTime);
        int textX = 50, textY = 50;
        int textWidth = 300, textHeight = 80;

        // Draw the semi-transparent background rectangle
        cv::Mat overlay;
        frame.copyTo(overlay);
        cv::rectangle(overlay, cv::Point(textX - 10, textY - 40), cv::Point(textX + textWidth, textY + textHeight - 10), cv::Scalar(0, 0, 0), cv::FILLED);
        double alpha = 0.6; // Transparency factor
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
    cap.release();
    SDL_GameControllerClose(controller);
    SDL_Quit();
    cv::destroyAllWindows();

    return 0;
}
