#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {
    // Open default camera (usually the first camera connected to the device)
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // Set the desired frame rate (increase based on what the camera supports)
    // double desired_fps = 60.0;  // Change this to the desired FPS
    // cap.set(cv::CAP_PROP_FPS, desired_fps);

    // Optionally reduce resolution to help with higher frame rates
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Create a window to display the output
    cv::namedWindow("Camera Output", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;
        // Capture each frame from the camera
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame." << std::endl;
            break;
        }

        // Add "test" text overlay on the frame
        cv::putText(frame, "60 FPS", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

        // Show the frame with the overlay
        cv::imshow("Camera Output", frame);

        // Press 'q' to exit
        if (cv::waitKey(30) == 'q') break;
    }

    // Release the camera and close any OpenCV windows
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
