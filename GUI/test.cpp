#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open the default camera (index 0)
    cv::VideoCapture cap(0);

    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // Set the camera resolution (optional)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Create a window to display the video
    cv::namedWindow("Camera Output", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;
        // Capture a new frame
        cap >> frame;

        // Check if the frame is empty (end of video)
        if (frame.empty()) {
            std::cerr << "Error: Could not capture frame." << std::endl;
            break;
        }

        // Display the frame in the window
        cv::imshow("Camera Output", frame);

        // Wait for 30ms and exit the loop if 'q' is pressed
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    // Release the camera and close all OpenCV windows
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
