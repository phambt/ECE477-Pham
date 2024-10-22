#include <SDL2/SDL.h>
#include <iostream>
#include <map>

// Deadzone threshold for joystick drift (adjust as needed)
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

// Function to handle button states (checking if multiple buttons are pressed)
void handleButtonStates(SDL_GameController* controller) {
    for (const auto& button : buttonLookup) {
        if (SDL_GameControllerGetButton(controller, static_cast<SDL_GameControllerButton>(button.first))) {
            std::cout << "Button pressed: " << button.second << std::endl;
        }
    }
}

// Function to handle joystick motion with deadzone logic
void handleJoystickMotion(SDL_GameController* controller) {
    int leftStickX = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
    int leftStickY = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
    int rightStickX = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX);
    int rightStickY = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY);

    // Apply deadzone filtering to prevent slight joystick drift
    if (abs(leftStickX) < JOYSTICK_DEADZONE) leftStickX = 0;
    if (abs(leftStickY) < JOYSTICK_DEADZONE) leftStickY = 0;
    if (abs(rightStickX) < JOYSTICK_DEADZONE) rightStickX = 0;
    if (abs(rightStickY) < JOYSTICK_DEADZONE) rightStickY = 0;

    // Print joystick values only if they have changed significantly
    if (leftStickX != 0 || leftStickY != 0) {
        std::cout << "Left Stick (X, Y): (" << leftStickX << ", " << leftStickY << ")" << std::endl;
    }

    if (rightStickX != 0 || rightStickY != 0) {
        std::cout << "Right Stick (X, Y): (" << rightStickX << ", " << rightStickY << ")" << std::endl;
    }
}

// Function to handle trigger motion
void handleTriggerMotion(SDL_GameController* controller) {
    int leftTrigger = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
    int rightTrigger = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

    if( leftTrigger){
        std::cout << "Left Trigger value: " << leftTrigger << std::endl;
    }
    if( rightTrigger ){
        std::cout << "Right Trigger value: " << rightTrigger << std::endl;  
    }
    
}

int main(int argc, char* argv[]) {
    // Initialize SDL2's game controller subsystem
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        std::cout << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    // Open the first available controller (index 0)
    SDL_GameController* controller = SDL_GameControllerOpen(0);
    if (controller == nullptr) {
        std::cout << "Could not open gamecontroller! SDL Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    // Main loop flag
    bool quit = false;

    // Event handler
    SDL_Event e;

    // Main loop for when the application is running
    while (!quit) {
        // Handle events on queue
        while (SDL_PollEvent(&e) != 0) {
            // User requests quit
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        // Handle button states, joysticks, and triggers
        handleButtonStates(controller);
        handleJoystickMotion(controller);
        handleTriggerMotion(controller);
        
        // Small delay to prevent spamming the output
        SDL_Delay(100);
    }

    // Clean up and close SDL
    SDL_GameControllerClose(controller);
    SDL_Quit();

    return 0;
}
