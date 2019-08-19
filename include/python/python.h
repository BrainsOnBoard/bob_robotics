#pragma once

// Third-party includes
#include "third_party/path.h"
#include "wrappy/wrappy.h"

// Numpy
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <vector>
#include <string>

namespace wrappy {

//! Run the specified python script
void
runFile(const filesystem::path &filePath);

//! Add the specified directory to the python path
void
appendToPythonPath(const filesystem::path &path);

//! Import a new python module
void
importModule(const std::string &moduleName);

#if PY_MAJOR_VERSION >= 3
inline void *
importNumpy()
{
    import_array(); // initialize C-API
    return nullptr;
}
#else
inline void
importNumpy()
{
    import_array(); // initialize C-API
}
#endif

// Type selector for numpy array conversion
template <typename T> struct NumpyType { const static NPY_TYPES type = NPY_NOTYPE; }; //Default
template <> struct NumpyType<double> { const static NPY_TYPES type = NPY_DOUBLE; };
template <> struct NumpyType<float> { const static NPY_TYPES type = NPY_FLOAT; };
template <> struct NumpyType<bool> { const static NPY_TYPES type = NPY_BOOL; };
template <> struct NumpyType<int8_t> { const static NPY_TYPES type = NPY_INT8; };
template <> struct NumpyType<int16_t> { const static NPY_TYPES type = NPY_SHORT; };
template <> struct NumpyType<int32_t> { const static NPY_TYPES type = NPY_INT; };
template <> struct NumpyType<int64_t> { const static NPY_TYPES type = NPY_INT64; };
template <> struct NumpyType<uint8_t> { const static NPY_TYPES type = NPY_UINT8; };
template <> struct NumpyType<uint16_t> { const static NPY_TYPES type = NPY_USHORT; };
template <> struct NumpyType<uint32_t> { const static NPY_TYPES type = NPY_ULONG; };
template <> struct NumpyType<uint64_t> { const static NPY_TYPES type = NPY_UINT64; };

//! A C++ wrapper for creating numpy arrays
class NumpyArray
  : public PythonObject
{
public:
    NumpyArray(PyObject *obj);

    //! Create a NumpyArray pointing to the contents of v
    template<class T>
    NumpyArray(std::vector<T> &v)
      : NumpyArray{ getObject(v) }
    {}

    //! Create a NumpyArray pointing to the contents of mat
    NumpyArray(cv::Mat &mat);

    //! Get the numpy type of the array
    static constexpr NPY_TYPES type();

private:
    template<class T>
    static PyObject *getObject(std::vector<T> &v)
    {
        importNumpy();
        const npy_intp size = v.size();
        return PyArray_SimpleNewFromData(1, &size, NumpyType<T>::type,
                                         static_cast<void *>(v.data()));
    }
}; // NumpyArray

} // wrappy

namespace BoBRobotics {
// Put all of wrappy's functions etc. into BoBRobotics::Python
namespace Python = wrappy;
} // BoBRobotics
