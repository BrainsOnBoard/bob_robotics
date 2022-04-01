#pragma once

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

// Third-party includes
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "third_party/optional.hpp"
#include "third_party/units.h"

// Standard C++ includes
#include <stdexcept>

// OpenCV
#include <opencv2/opencv.hpp>


namespace BoBRobotics {
namespace Navigation {
void
addAlgorithmClasses(pybind11::module_ &m);
} // Navigation
} // BoBRobotics


namespace pybind11 {
namespace detail {
// Using optionals is a convenient way to allow passing None values from Python
template <typename T>
struct type_caster<std::experimental::optional<T>>
    : public optional_caster<std::experimental::optional<T>> {};

template<>
struct type_caster<std::experimental::nullopt_t>
    : public void_caster<std::experimental::nullopt_t> {};

template<>
struct type_caster<cv::Size>
{
    PYBIND11_TYPE_CASTER(cv::Size, _("cvSize"));

    // For converting from a Python tuple to a cv::Size
    bool load(handle src, bool)
    {
        return PyArg_ParseTuple(src.ptr(), "ii", &value.width, &value.height);
    }
};

// Coerce all angle-unit types into radians
template<class T>
struct type_caster<T, std::enable_if_t<units::traits::is_angle_unit<T>::value>>
{
    PYBIND11_TYPE_CASTER(units::angle::radian_t, _("radians"));

    static handle cast(units::angle::radian_t src, return_value_policy /* policy */, handle /* parent */)
    {
        return PyFloat_FromDouble(src.value());
    }
};

template<>
struct type_caster<cv::Mat>
{
    PYBIND11_TYPE_CASTER(cv::Mat, _("cvMat"));

    bool load(handle src, bool);
};
} // detail
} // pybind11
