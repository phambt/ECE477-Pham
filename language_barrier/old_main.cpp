#include <Python.h>
#include <iostream>
#include <vector>
#include <unistd.h>

// Global Python state
bool isPythonInitialized = false;



// Global objects
PyObject* pModule = nullptr;
PyObject* pFunc = nullptr;

// void initializePythonOnce() {
void initializePythonOnce() {
    if (!isPythonInitialized) {
        Py_Initialize();
        isPythonInitialized = true;

        // Append the specific path where jetson.py exists
        PyObject* sysPath = PySys_GetObject("path");
        PyList_Append(sysPath, PyUnicode_FromString("/home/ece477/Desktop/477/ikpy"));

        // Debugging: Print Python sys.path
        PyObject* repr = PyObject_Repr(sysPath);
        PyObject* str = PyUnicode_AsEncodedString(repr, "utf-8", "~E~");
        const char* bytes = PyBytes_AS_STRING(str);
        std::cout << "Python sys.path: " << bytes << std::endl;
        Py_XDECREF(repr);
        Py_XDECREF(str);
    }
}

void finalizePythonOnce() {
    if (isPythonInitialized) {
        Py_XDECREF(pFunc);
        Py_XDECREF(pModule);
        Py_Finalize();
        isPythonInitialized = false;
    }
}

std::vector<double> call_getAngle(double x, double y, double z) {
    std::vector<double> jointAngles;

    if (!pFunc) {
        std::cerr << "Python function not initialized." << std::endl;
        return jointAngles;
    }

    PyObject* pArgs = PyTuple_Pack(3, PyFloat_FromDouble(x), PyFloat_FromDouble(y), PyFloat_FromDouble(z));
    PyObject* pValue = PyObject_CallObject(pFunc, pArgs);

    Py_XDECREF(pArgs); // Always clean up

    if (pValue) {
        PyObject* pList = PyObject_CallMethod(pValue, "tolist", nullptr);
        if (pList && PyList_Check(pList)) {
            for (Py_ssize_t i = 0; i < PyList_Size(pList); ++i) {
                PyObject* item = PyList_GetItem(pList, i);
                if (PyFloat_Check(item)) {
                    jointAngles.push_back(PyFloat_AsDouble(item));
                }
            }
        }
        Py_XDECREF(pList);
        Py_DECREF(pValue);
    } else {
        PyErr_Print();
    }

    return jointAngles;
}

int main() {
    initializePythonOnce();

    for (int i = 0; i < 1000; ++i) { // Simulate repeated calls
        double x = 0.2, y = 0.2, z = 0.0;
        auto angles = call_getAngle(x, y, z);
        if (!angles.empty()) {
            std::cout << "Call " << i << ": " << angles[0] << std::endl;
        }
    }

    finalizePythonOnce();
    return 0;
}
