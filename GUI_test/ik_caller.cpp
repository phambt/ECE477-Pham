#include <Python.h>
#include <iostream>
#include <vector>
#include <unistd.h>

std::vector<double> call_getAngle(double x, double y, double z) {
    std::vector<double> jointAngles;

    // Change the current working directory to the 'ikpy' folder
    if (chdir("/home/ece477/Desktop/477/ikpy") != 0) {
        perror("chdir failed");
        return jointAngles; // Return an empty vector on error
    }

    // Initialize the Python interpreter
    Py_Initialize();

    // Add the current directory (now 'ikpy') to Python's module search path
    PyObject* sysPath = PySys_GetObject("path");
    PyList_Append(sysPath, PyUnicode_FromString("."));

    // Load the Python module (jetson.py)
    const char* scriptName = "jetson";
    PyObject* pName = PyUnicode_DecodeFSDefault(scriptName);
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != nullptr) {
        PyObject* pFunc = PyObject_GetAttrString(pModule, "getAngle");

        if (pFunc && PyCallable_Check(pFunc)) {
            PyObject* pArgs = PyTuple_Pack(3, 
                PyFloat_FromDouble(x), 
                PyFloat_FromDouble(y), 
                PyFloat_FromDouble(z));

            PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);

            if (pValue != nullptr) {
                // Check if the returned value is a NumPy array
                if (PyObject_HasAttrString(pValue, "tolist")) {
                    // Convert NumPy array to Python list using tolist()
                    PyObject* pList = PyObject_CallMethod(pValue, "tolist", nullptr);
                    Py_DECREF(pValue); // Free the NumPy array

                    if (pList && PyList_Check(pList)) {
                        for (Py_ssize_t i = 0; i < PyList_Size(pList); ++i) {
                            PyObject* item = PyList_GetItem(pList, i); // Borrowed reference
                            jointAngles.push_back(PyFloat_AsDouble(item));
                        }
                    } else {
                        std::cerr << "Error: Converted object is not a list!" << std::endl;
                        PyErr_Print();
                    }
                    Py_XDECREF(pList); // Free the converted Python list
                } else {
                    std::cerr << "Error: Returned object does not have 'tolist' method!" << std::endl;
                }
            } else {
                std::cerr << "Error: Python function call failed!" << std::endl;
                PyErr_Print();
            }

            Py_DECREF(pFunc);
        } else {
            std::cerr << "Error: Python function 'getAngle' not callable or missing!" << std::endl;
            PyErr_Print();
        }

        Py_DECREF(pModule);
    } else {
        std::cerr << "Error: Python module 'jetson' could not be loaded!" << std::endl;
        PyErr_Print();
    }

    Py_Finalize();
    return jointAngles;
}

// int main() {
//     double x = 0.2, y = 0.2, z = 0.0;
//     std::vector<double> jointAngles = call_getAngle(x, y, z);

//     if (!jointAngles.empty()) {
//         std::cout << "Python function returned the following joint angles:" << std::endl;
//         for (double angle : jointAngles) {
//             std::cout << angle << " ";
//         }
//         std::cout << std::endl;
//     } else {
//         std::cerr << "Failed to retrieve joint angles." << std::endl;
//     }

//     return 0;
// }
