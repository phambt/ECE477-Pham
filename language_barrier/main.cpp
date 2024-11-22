#include <pybind11/embed.h> // Everything needed for embedding
#include <iostream>
#include <vector>

namespace py = pybind11;

std::vector<double> getAngle(double x, double y, double z) {
    std::cout << "Initializing Python..." << std::endl;
    py::scoped_interpreter guard{};

    std::cout << "Appending to sys.path..." << std::endl;
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")("/home/ece477/Desktop/477/ikpy");  // Use the absolute path

    py::list paths = sys.attr("path");
    for (auto path : paths) {
        std::cout << py::str(path).cast<std::string>() << std::endl;
    }

    sys.attr("path").attr("append")("../ikpy");

    std::cout << "Importing jetson module..." << std::endl;
    py::module_ jetson = py::module_::import("jetson");

    std::cout << "Calling getAngle function..." << std::endl;
    py::list result = jetson.attr("getAngle")(x, y, z);

    std::cout << "Parsing result..." << std::endl;


    std::vector<double> angles;
    for (auto item : result) {
        angles.push_back(item.cast<double>());
    }
    return angles;
}

int main() {
    try {
        double x = 0.5, y = 0.2, z = 0.3;
        std::vector<double> angles = getAngle(x, y, z);

        std::cout << "Joint angles: ";
        for (double angle : angles) {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
