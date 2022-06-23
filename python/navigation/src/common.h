#pragma once

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

// BoB robotics includes
#include "navigation/image_database.h"

// Third-party includes
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "range/v3/algorithm.hpp"
#include "range/v3/iterator.hpp"
#include "range/v3/view.hpp"
#include "third_party/optional.hpp"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <stdexcept>

void
checkPythonErrors();

template<class T>
auto
toRange(const pybind11::iterable &iterable)
{
    const auto doCast = [](const pybind11::handle &handle) { return handle.cast<T>(); };
    return ranges::views::transform(iterable, doCast);
}

namespace BoBRobotics {
namespace Navigation {

void
addAlgorithmClasses(pybind11::module_ &m);

void
addDatabaseClass(pybind11::module_ &m);

inline pybind11::object
getColumn(const pybind11::object &df, const char *name)
{
    auto column = df[name];

    if (pybind11::hasattr(column, "to_list")) {
        return column.attr("to_list")();
    }

    return pybind11::make_tuple(column);
}

struct ImageSet {
    pybind11::array images;
    std::experimental::optional<pybind11::object> dataFrame;
    bool singleImage = false;

    ImageSet() = default;

    ImageSet(pybind11::object imageSet)
    {
        init(std::move(imageSet));
    }

    void init(pybind11::object imageSet)
    {
        using namespace pybind11;

        object imagesObj;
        if (hasattr(imageSet, "iloc")) {
            // ...then it's a DataFrame/Series
            imagesObj = imageSet["image"];

            // If it's a column with multiple values
            if (hasattr(imagesObj, "to_list")) {
                imagesObj = imagesObj.attr("to_list")();
            }

            dataFrame.emplace(std::move(imageSet));
        } else {
            imagesObj = std::move(imageSet);
        }

        static auto atLeast2d = module::import("numpy").attr("atleast_2d");
        images = atLeast2d(std::move(imagesObj));
        BOB_ASSERT(images.ndim() <= 3);
        if (images.ndim() == 2) {
            images = images.reshape(array::ShapeContainer({ 1, images.shape(0), images.shape(1) }));
            singleImage = true;
        }
    }

    pybind11::object
    getColumn(const char *name) const
    {
        return BoBRobotics::Navigation::getColumn(*dataFrame, name);
    }

    pybind11::ssize_t
    size() const
    {
        return pybind11::len(images);
    }

    auto toRange() const
    {
        return ::toRange<cv::Mat>(images);
    }
};
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

/*
 * NB: OpenCV's convention is width x height as opposed to numpy's
 * height x width, so we swap the order when going from C++ -> Python and vice
 * versa.
 */
template<>
struct type_caster<cv::Size>
{
    PYBIND11_TYPE_CASTER(cv::Size, _("cvSize"));

    bool load(handle src, bool)
    {
        std::tie(value.height, value.width) = src.cast<std::pair<int, int>>();
        return true;
    }

    static handle cast(const cv::Size &size, return_value_policy /* policy */, handle /* parent */)
    {
        return make_tuple(size.height, size.width).release();
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

// Coerce all length-unit types into metres
template<class T>
struct type_caster<T, std::enable_if_t<units::traits::is_length_unit<T>::value>>
{
    PYBIND11_TYPE_CASTER(units::length::meter_t, _("metres"));

    static handle cast(units::length::meter_t src, return_value_policy /* policy */, handle /* parent */)
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
        using namespace pybind11::literals;

        const auto &p = entry.pose;
        dict dict{ "x"_a = p.x(), "y"_a = p.y(), "z"_a = p.z(),
                   "yaw"_a = p.yaw(), "pitch"_a = p.pitch(), "roll"_a = p.roll(),
                   "filepath"_a = entry.path.str() };

        // Add extra fields
        for (const auto &item : entry.extraFields) {
            dict[item.first.c_str()] = item.second;
        }

        return dict.release();
    }
};

template<>
struct type_caster<BoBRobotics::Navigation::ImageSet>
{
    PYBIND11_TYPE_CASTER(BoBRobotics::Navigation::ImageSet, _("ImageSet"));

    bool load(handle src, bool)
    {
        value.init(src.cast<object>());
        return true;
    }
};
} // detail
} // pybind11
