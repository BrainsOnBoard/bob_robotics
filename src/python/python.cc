// BoB robotics includes
#include "python/python.h"
#include "common/macros.h"

// Standard C includes
#include <cstdio>

// Standard C++ includes
#include <stdexcept>

namespace wrappy {

void
runFile(const filesystem::path &filePath)
{
    /*
     * Apparently we need to read in binary mode so that on Windows, line
     * endings are handled correctly.
     */
    auto filp = fopen(filePath.str().c_str(), "rb");

    // filp will automatically be closed
    if (PyRun_SimpleFileEx(filp, filePath.str().c_str(), true) != 0) {
        throw std::runtime_error("Error executing python script: " + filePath.str());
    }
}

void
appendToPythonPath(const filesystem::path &path)
{
    const std::string runStr = "import sys\n"
                               "sys.path.append('" +
                               path.str() + "')";
    PyRun_SimpleString(runStr.c_str());
}

Object
importModule(const std::string &moduleName)
{
    Object module{ Object::owning{},
                   PyImport_Import(construct(moduleName).get()) };
    if (!module) {
        throw std::runtime_error("Could not import module " + moduleName);
    }
    return module;
}

NPY_TYPES
opencvTypeToNumpyType(int cvType)
{
    switch (cvType & CV_MAT_DEPTH_MASK) {
    case CV_8U:
        return NPY_UINT8;
    case CV_8S:
        return NPY_INT8;
    case CV_16U:
        return NPY_UINT16;
    case CV_16S:
        return NPY_INT16;
    case CV_32S:
        return NPY_INT32;
    case CV_32F:
        return NPY_FLOAT32;
    case CV_64F:
        return NPY_FLOAT64;
    default:
        return NPY_NOTYPE;
    }
}

PyObject *
getObject(cv::Mat &mat)
{
    importNumpy();
    std::vector<npy_intp> size;
    size.reserve(mat.dims);
    for (int i = 0; i < mat.dims; i++) {
        size.push_back(mat.size[i]);
    }

    // Colour channels are an extra dimension
    if (mat.channels() > 1) {
        size.push_back(mat.channels());
    }

    auto npType = opencvTypeToNumpyType(mat.type());
    return PyArray_SimpleNewFromData(size.size(), size.data(), npType,
                                     static_cast<void *>(mat.data));
}

NumpyArray::NumpyArray(PyObject *obj)
  : Object{ Object::owning{}, obj }
{
    if (!obj) {
        throw std::runtime_error("Could not create numpy array");
    }
}

NumpyArray::NumpyArray(cv::Mat &mat)
  : NumpyArray{ wrappy::getObject(mat) }
{}

} // wrappy
