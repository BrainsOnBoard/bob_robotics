#include "common.h"

// Python includes
#include <numpy/arrayobject.h>

// BoB robotics includes
#include "common/macros.h"
#include "common/main.h"

// Standard C++ includes
#include <array>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;
namespace py = pybind11;

namespace {
py::dtype
cvTypeToNumpy(int depth)
{
    switch (depth) {
    case CV_8U: return py::dtype::of<uint8_t>();
    case CV_8S: return py::dtype::of<int8_t>();
    case CV_16U: return py::dtype::of<uint16_t>();
    case CV_16S: return py::dtype::of<int16_t>();
    case CV_32S: return py::dtype::of<int32_t>();
    case CV_32F: return py::dtype::of<float>();
    case CV_64F: return py::dtype::of<double>();
    default:
        throw std::invalid_argument("Unsupported data type.");
    }
}

py::capsule
makeCapsule(cv::Mat m)
{
    return { new cv::Mat(std::move(m)),
             [](void *v) { delete reinterpret_cast<cv::Mat *>(v); } };
}
} // anonymous namespace

namespace pybind11 {
namespace detail {
bool
type_caster<cv::Mat>::load(handle src, bool)
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

handle
type_caster<cv::Mat>::cast(const cv::Mat &m, return_value_policy, handle)
{
    // Handle empty matrices
    if (m.empty()) {
        return array().release();
    }

    // TODO: Handle non-continuous matrices
    BOB_ASSERT(m.isContinuous());

    // TODO: Handle colour images (will involve copying, as with non-continuous mats)
    BOB_ASSERT(m.channels() == 1);

    return array(cvTypeToNumpy(m.type()), { m.rows, m.cols }, m.data,
                 makeCapsule(m)).release();
}
} // detail
} // pybind11

PYBIND11_MODULE(_navigation, m)
{
    // Initialise plog
    initLogging();

    // Initialise numpy
    if (_import_array() < 0) {
        throw std::runtime_error{ "numpy.core.multiarray failed to import" };
    }

    addAlgorithmClasses(m);
    addDatabaseClass(m);
}
