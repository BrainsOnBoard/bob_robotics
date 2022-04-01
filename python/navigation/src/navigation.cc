#include "common.h"

// Python includes
#include <numpy/arrayobject.h>

// BoB robotics includes
#include "common/main.h"
#include "common/macros.h"

namespace pybind11 {
namespace detail {
bool type_caster<cv::Mat>::load(handle src, bool)
{
    auto *obj = src.ptr();
    if (!PyArray_Check(obj)) {
        return false;
    }

    auto *arr = reinterpret_cast<PyArrayObject *>(obj);

    /*
     * It would be totally trivial to add other types, but we're only
     * interested in uint8-type images for now.
     */
    int cvType;
    switch (PyArray_TYPE(arr)) {
    case NPY_UINT8:
        cvType = CV_8U;
        break;
    default:
        throw std::runtime_error{ "Only uint8-type arrays are allowed" };
    }

    /*
        * Note that we are taking a reference to arr's data, rather than
        * copying it. We only support 2D (i.e. greyscale) images.
        */
    BOB_ASSERT(PyArray_NDIM(arr) == 2);
    const cv::Size size{ (int) PyArray_DIM(arr, 1), (int) PyArray_DIM(arr, 0) };
    value = cv::Mat(size, cvType, PyArray_DATA(arr));

    return true;
}
} // detail
} // pybind11

PYBIND11_MODULE(_navigation, m)
{
    // Initialise plog
    BoBRobotics::initLogging();

    // Initialise numpy
    if (_import_array() < 0) {
        throw std::runtime_error{ "numpy.core.multiarray failed to import" };
    }

    BoBRobotics::Navigation::addAlgorithmClasses(m);
}
