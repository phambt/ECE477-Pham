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
        // Add your 'ikpy' folder to Python's module search path
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
        initializePython();

        // Load the Python module (jetson.py)
        PyObject* pName = PyUnicode_DecodeFSDefault("jetson");
        PyObject* pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if (!pModule) {
            std::cerr << "Error: Python module 'jetson' could not be loaded!" << std::endl;
            PyErr_Print();
            return jointAngles;
        }

        // Get the 'getAngle' function
        PyObject* pFunc = PyObject_GetAttrString(pModule, "getAngle");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            std::cerr << "Error: Python function 'getAngle' not callable or missing!" << std::endl;
            PyErr_Print();
            Py_XDECREF(pModule);
            return jointAngles;
        }

        // Prepare arguments
        PyObject* pArgs = PyTuple_Pack(3, 
            PyFloat_FromDouble(x), 
            PyFloat_FromDouble(y), 
            PyFloat_FromDouble(z));

        // Call the function
        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pArgs);

        if (pValue) {
            // Convert NumPy array or Python list to std::vector
            if (PyObject_HasAttrString(pValue, "tolist")) {
                PyObject* pList = PyObject_CallMethod(pValue, "tolist", nullptr);
                if (pList && PyList_Check(pList)) {
                    for (Py_ssize_t i = 0; i < PyList_Size(pList); ++i) {
                        PyObject* item = PyList_GetItem(pList, i); // Borrowed reference
                        jointAngles.push_back(PyFloat_AsDouble(item));
                    }
                } else {
                    std::cerr << "Error: Returned object is not a list!" << std::endl;
                    PyErr_Print();
                }
                Py_XDECREF(pList);
            } else {
                std::cerr << "Error: Returned object does not have 'tolist' method!" << std::endl;
            }
            Py_DECREF(pValue);
        } else {
            std::cerr << "Error: Python function call failed!" << std::endl;
            PyErr_Print();
        }

        Py_XDECREF(pFunc);
        Py_DECREF(pModule);

    } catch (const std::exception& e) {
        std::cerr << "Exception in call_getAngle: " << e.what() << std::endl;
    }

    return jointAngles;
}