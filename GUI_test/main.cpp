#include <opencv2/opencv.hpp>
#include <SDL2/SDL.h>
#include <iostream>
#include <map>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "ik_caller.cpp"

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
    double position[3] = {0, 0, 0.5};
    double previousPosition[3] = {0, 0, 0.5};

    while (!quit) {
        // Handle SDL events
        SDL_Event e;
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }
        double xAxis = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
        double yAxis = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);        

        if (std::abs(xAxis) > JOYSTICK_DEADZONE) {
            position[0] = position[0] + xAxis / 10000000; // Scale down for display
        } 
        if (std::abs(yAxis) > JOYSTICK_DEADZONE) {
            position[1] = position[1] - yAxis / 10000000; // Scale down for display
        } 


        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_UP)) {
            position[2] += 0.001; // Increase z
        }
        if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
            position[2] -= 0.001; // Decrease z
        }
        auto lastCallTime = std::chrono::steady_clock::now();

        // Check if position has changed
        if (position[0] != previousPosition[0] || 
            position[1] != previousPosition[1] || 
            position[2] != previousPosition[2]) {
            
            // Print updated position
            std::cout << "[" << position[0] << ", " << position[1] << ", " << position[2] << "]" << std::endl;

            // Update previousPosition
            previousPosition[0] = position[0];
            previousPosition[1] = position[1];
            previousPosition[2] = position[2];

            double x = position[0], y = position[1], z = position[2]; // Example coordinates

            // Call the function from Folder1/main.cpp
            std::vector<double> jointAngles = call_getAngle(x, y, z);

            if (!jointAngles.empty()) {
                std::cout << "Joint angles returned from Python function:" << std::endl;
                for (double angle : jointAngles) {
                    std::cout << angle << " ";
                }
                std::cout << std::endl;
            } else {
                std::cerr << "Failed to retrieve joint angles!" << std::endl;
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
        cv::waitKey(300);
    }

    // Clean up
    cam1.release();
    cam2.release();
    SDL_GameControllerClose(controller);
    SDL_Quit();
    cv::destroyAllWindows();


    // double x = 0.2, y = 0.2, z = 0.0; // Example coordinates

    // // Call the function from Folder1/main.cpp
    // std::vector<double> jointAngles = call_getAngle(x, y, z);

    // if (!jointAngles.empty()) {
    //     std::cout << "Joint angles returned from Python function:" << std::endl;
    //     for (double angle : jointAngles) {
    //         std::cout << angle << " ";
    //     }
    //     std::cout << std::endl;
    // } else {
    //     std::cerr << "Failed to retrieve joint angles!" << std::endl;
    // }

    return 0;
}
