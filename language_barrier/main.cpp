#include <Python.h>
#include <iostream>
#include <string>

int main() {
    // Initialize the Python interpreter
    Py_Initialize();

    // Add the current directory to the Python module search path
    PyObject* sysPath = PySys_GetObject("path");
    PyList_Append(sysPath, PyUnicode_FromString("."));

    // Name of the Python file without the `.py` extension
    const char* scriptName = "script";
    PyObject* pName = PyUnicode_DecodeFSDefault(scriptName);
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);


    if (pModule != nullptr) {
        // Get the function from the module
        PyObject* pFunc = PyObject_GetAttrString(pModule, "greet");

        // Ensure the function is callable
        if (PyCallable_Check(pFunc)) {
            // Create arguments for the function
            PyObject* pArgs = PyTuple_Pack(1, PyUnicode_FromString("Anja"));

            // Call the function
            PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);

            if (pValue != nullptr) {
                // Print the result from Python
                std::cout << "Python function returned: "
                          << PyUnicode_AsUTF8(pValue) << std::endl;
                Py_DECREF(pValue);
            } else {
                PyErr_Print();
            }
        } else {
            PyErr_Print();
        }

        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    } else {
        PyErr_Print();
    }

    // Finalize the Python interpreter
    Py_Finalize();

    return 0;
}
