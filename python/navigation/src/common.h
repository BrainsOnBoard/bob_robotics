#pragma once

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

// BoB robotics includes
#include "navigation/image_database.h"

// Third-party includes
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "third_party/optional.hpp"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <stdexcept>


namespace BoBRobotics {
namespace Navigation {
void
addAlgorithmClasses(pybind11::module_ &m);

void
addDatabaseClass(pybind11::module_ &m);
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
    static handle cast(const cv::Mat &mat, return_value_policy rvp, handle parent);
};

template<>
struct type_caster<BoBRobotics::Navigation::ImageDatabase::Entry>
{
    PYBIND11_TYPE_CASTER(BoBRobotics::Navigation::ImageDatabase::Entry, _("DatabaseEntry"));

    static handle cast(const BoBRobotics::Navigation::ImageDatabase::Entry &entry, return_value_policy /* policy */, handle /* parent */)
    {
        using namespace units::angle;
        using namespace units::length;

        auto *dict = PyDict_New();
        BOB_ASSERT(dict);

        auto setItem = [&dict](const char *key, PyObject *obj) {
            BOB_ASSERT(PyDict_SetItem(dict, PyUnicode_FromString(key), obj) == 0);
        };
        auto setItemLen = [&](const char *key, meter_t length) {
            setItem(key, PyFloat_FromDouble(length.value()));
        };
        auto setItemAng = [&](const char *key, radian_t angle) {
            setItem(key, PyFloat_FromDouble(angle.value()));
        };
        auto setItemStr = [&](const char *key, const std::string &str) {
            setItem(key,
                    PyUnicode_FromStringAndSize(str.c_str(), str.size()));
        };

        setItemLen("x", entry.pose.x());
        setItemLen("y", entry.pose.y());
        setItemLen("z", entry.pose.z());
        setItemAng("yaw", entry.pose.yaw());
        setItemAng("pitch", entry.pose.pitch());
        setItemAng("roll", entry.pose.roll());
        setItemStr("filepath", entry.path.str());
        for (const auto &item : entry.extraFields) {
            setItemStr(item.first.c_str(), item.second);
        }

        return dict;
    }
};
} // detail
} // pybind11
