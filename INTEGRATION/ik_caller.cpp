#include <Python.h>
#include <iostream>
#include <vector>
#include <unistd.h>

// Global Python state
bool isPythonInitialized = false;

void initializePython() {
    if (!isPythonInitialized) {
        Py_Initialize();
        isPythonInitialized = true;
        PyObject* sysPath = PySys_GetObject("path");
        PyList_Append(sysPath, PyUnicode_FromString("/home/ece477/Desktop/477/ikpy"));
    }
}

void finalizePython() {
    if (isPythonInitialized) {
        Py_Finalize();
        isPythonInitialized = false;
    }
}


std::vector<double> call_getAngle(double x, double y, double z) {
    std::vector<double> jointAngles;

    try {
        initializePython(); // Ensure Python is initialized

        PyObject* pName = PyUnicode_DecodeFSDefault("jetson");
        if (!pName) {
            std::cerr << "Failed to create Python module name 'jetson'." << std::endl;
            PyErr_Print();
            return jointAngles;
        }

        PyObject* pModule = PyImport_Import(pName);
        Py_DECREF(pName);
        if (!pModule) {
            std::cerr << "Failed to load Python module 'jetson'." << std::endl;
            PyErr_Print();
            return jointAngles;
        }

        PyObject* pFunc = PyObject_GetAttrString(pModule, "getAngle");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            std::cerr << "Python function 'getAngle' not callable or missing!" << std::endl;
            PyErr_Print();
            Py_XDECREF(pFunc);
            Py_DECREF(pModule);
            return jointAngles;
        }

        PyObject* pArgs = PyTuple_Pack(3, 
            PyFloat_FromDouble(x), 
            PyFloat_FromDouble(y), 
            PyFloat_FromDouble(z));
        if (!pArgs) {
            std::cerr << "Failed to create arguments for 'getAngle'." << std::endl;
            PyErr_Print();
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            return jointAngles;
        }

        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pArgs);
        if (!pValue) {
            std::cerr << "Error calling Python function 'getAngle'." << std::endl;
            PyErr_Print();
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            return jointAngles;
        }

        PyObject* pList = PyObject_CallMethod(pValue, "tolist", nullptr);
        if (pList && PyList_Check(pList)) {
            for (Py_ssize_t i = 0; i < PyList_Size(pList); ++i) {
                PyObject* item = PyList_GetItem(pList, i);
                if (PyFloat_Check(item)) {
                    jointAngles.push_back(PyFloat_AsDouble(item));
                }
            }
            Py_DECREF(pList);
        } else {
            std::cerr << "Returned object is not a valid list or 'tolist' failed." << std::endl;
            PyErr_Print();
        }

        Py_DECREF(pValue);
        Py_DECREF(pFunc);
        Py_DECREF(pModule);

    } catch (const std::exception& e) {
        std::cerr << "Exception in call_getAngle: " << e.what() << std::endl;
    }

    return jointAngles;
}




// std::vector<double> call_getPosition(double angle1, double angle2, double angle3, double angle4, double angle5) {
//     std::vector<double> position;

//     try {
//         initializePython();

//         // Load the Python module (jetson.py)
//         PyObject* pName = PyUnicode_DecodeFSDefault("jetson");
//         PyObject* pModule = PyImport_Import(pName);
//         Py_XDECREF(pName);

//         if (!pModule) {
//             std::cerr << "Error: Python module 'jetson' could not be loaded!" << std::endl;
//             PyErr_Print();
//             return position;
//         }

//         // Get the 'getPosition' function
//         PyObject* pFunc = PyObject_GetAttrString(pModule, "getPosition");
//         if (!pFunc || !PyCallable_Check(pFunc)) {
//             std::cerr << "Error: Python function 'getPosition' not callable or missing!" << std::endl;
//             PyErr_Print();
//             Py_XDECREF(pModule);
//             return position;
//         }

//         // Prepare arguments
//         PyObject* pArgs = PyTuple_Pack(5, 
//             PyFloat_FromDouble(angle1), 
//             PyFloat_FromDouble(angle2), 
//             PyFloat_FromDouble(angle3), 
//             PyFloat_FromDouble(angle4), 
//             PyFloat_FromDouble(angle5));

//         // Call the function
//         PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
//         Py_XDECREF(pArgs);

//         if (pValue) {
//             // Convert Python list to std::vector
//             if (PyList_Check(pValue)) {
//                 for (Py_ssize_t i = 0; i < PyList_Size(pValue); ++i) {
//                     PyObject* item = PyList_GetItem(pValue, i); // Borrowed reference
//                     position.push_back(PyFloat_AsDouble(item));
//                 }
//             } else {
//                 std::cerr << "Error: Returned object is not a list!" << std::endl;
//                 PyErr_Print();
//             }
//             Py_XDECREF(pValue);
//         } else {
//             std::cerr << "Error: Python function call failed!" << std::endl;
//             PyErr_Print();
//         }

//         Py_XDECREF(pFunc);
//         Py_XDECREF(pModule);

//     } catch (const std::exception& e) {
//         std::cerr << "Exception in call_getPosition: " << e.what() << std::endl;
//     }

//     return position;
// }
