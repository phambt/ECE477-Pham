#include <iostream>
#include "ik_caller.cpp" // Include the original file with the function definition

int main() {
    double x = 0.2, y = 0.2, z = 0.0; // Example coordinates

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

    return 0;
}
