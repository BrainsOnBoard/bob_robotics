// BoB robotics includes
#include "common/main.h"
#include "navigation/image_database.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// Third-party includes
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "third_party/units.h"

// Python includes
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

// Standard C++ includes
#include <stdexcept>

namespace py = pybind11;
using namespace py::literals;
using namespace BoBRobotics::Navigation;
using namespace units::angle;

using PerfectMemoryType = PerfectMemoryRotater<>;
using InfoMaxType = InfoMaxRotater<>;

template<class T>
using optional = std::experimental::optional<T>;
constexpr std::experimental::nullopt_t nullopt{ std::experimental::nullopt_t::init() };

// Using optionals is a convenient way to allow passing None values from Python
namespace pybind11 { namespace detail {
    template <typename T>
    struct type_caster<optional<T>> : optional_caster<optional<T>> {};
}}

namespace pybind11 {
namespace detail {
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
    PYBIND11_TYPE_CASTER(radian_t, _("radians"));

    static handle cast(radian_t src, return_value_policy /* policy */, handle /* parent */)
    {
        return PyFloat_FromDouble(src.value());
    }
};

template<>
struct type_caster<cv::Mat>
{
    PYBIND11_TYPE_CASTER(cv::Mat, _("cvMat"));

    bool load(handle src, bool)
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
};
}
} // namespace pybind11::detail

template<class T>
class PyAlgoWrapperBase
{
public:
    template<class... Ts>
    PyAlgoWrapperBase(Ts&&... args)
      : m_Algo(std::forward<Ts>(args)...)
    {}

    auto getHeading(const cv::Mat &img) const
    {
        return std::get<0>(getRIDFData(img));
    }

    auto getRIDFData(const cv::Mat &img) const
    {
        return m_Algo.getHeading(getResizedMat(img));
    }

    auto test(const cv::Mat &img) const
    {
        return m_Algo.test(getResizedMat(img));
    }

    void train(const cv::Mat &img)
    {
        m_Algo.train(getResizedMat(img));
    }

    void trainRoute(const char *dbPath)
    {
        const ImageDatabase database{ dbPath };
        BOB_ASSERT(!database.empty());

        for (const auto &entry : database) {
            train(entry.load());

            // Handle e.g. Ctrl+C during training
            if (PyErr_CheckSignals()) {
                return;
            }
        }
    }

protected:
    T m_Algo;

private:
    mutable cv::Mat m_Resized;

    const auto &getResizedMat(const cv::Mat &img) const
    {
        const auto size = m_Algo.getUnwrapResolution();
        if (img.size() != size) {
            cv::resize(img, m_Resized, size);
            return m_Resized;
        }

        // No resizing needed
        return img;
    }
};

template<class T>
class PyAlgoWrapper
  : public PyAlgoWrapperBase<T>
{
public:
    template<class... Ts>
    PyAlgoWrapper(Ts&&... args)
      : PyAlgoWrapperBase<T>(std::forward<Ts>(args)...)
    {}
};


template<>
class PyAlgoWrapper<InfoMaxType>
  : public PyAlgoWrapperBase<InfoMaxType>
{
public:
    PyAlgoWrapper(const cv::Size &size, float learningRate,
                  float tanhScalingFactor, Eigen::MatrixXf weights)
      : PyAlgoWrapperBase<InfoMaxType>(size, std::move(weights), learningRate, tanhScalingFactor)
    {}

    PyAlgoWrapper(const cv::Size &size, float learningRate,
                  float tanhScalingFactor)
      : PyAlgoWrapper(size, learningRate, tanhScalingFactor, generateInitialWeights(size).first)
    {}

    const auto &getWeights() const { return m_Algo.getWeights(); }

    static std::pair<Eigen::MatrixXf, unsigned>
    generateInitialWeights(const cv::Size &size,
                           optional<int> numHidden = nullopt,
                           optional<unsigned> seedArg = nullopt)
    {
        const auto seed = seedArg ? seedArg.value() : std::random_device()();
        const auto numInput = size.width * size.height;
        auto weights = InfoMax<>::generateInitialWeights(numInput,
                                                         numHidden.value_or(numInput),
                                                         seed);
        return std::make_pair(weights, seed);
    }
};

template<class Algo>
auto
addAlgo(py::handle scope, const char *name)
{
    using T = PyAlgoWrapper<Algo>;
    return py::class_<T>(scope, name)
            .def("get_heading", &T::getHeading)
            .def("get_ridf_data", &T::getRIDFData)
            .def("test", &T::test)
            .def("train", &T::train)
            .def("train_route", &T::trainRoute);
}

PYBIND11_MODULE(_navigation, m)
{
    // Initialise plog
    BoBRobotics::initLogging();

    // Initialise numpy
    if (_import_array() < 0) {
        throw std::runtime_error{ "numpy.core.multiarray failed to import" };
    }

    // Add various algorithms as Python classes
    addAlgo<PerfectMemoryType>(m, "PerfectMemory")
            .def(py::init<const cv::Size &>());
    addAlgo<InfoMaxType>(m, "InfoMax")
            .def(py::init<const cv::Size &, float, float>(),
                 "size"_a,
                 "learning_rate"_a = INFOMAX_DEFAULT_LEARNING_RATE,
                 "tanh_scaling_factor"_a = DEFAULT_TANH_SCALING_FACTOR)
            .def(py::init<const cv::Size &, float, float, Eigen::MatrixXf>(),
                 "size"_a,
                 "learning_rate"_a = INFOMAX_DEFAULT_LEARNING_RATE,
                 "tanh_scaling_factor"_a = DEFAULT_TANH_SCALING_FACTOR,
                 "weights"_a)
            .def("get_weights", &PyAlgoWrapper<InfoMaxType>::getWeights)
            .def_static("generate_initial_weights",
                        &PyAlgoWrapper<InfoMaxType>::generateInitialWeights,
                        "size"_a,
                        "num_hidden"_a = optional<int>{},
                        "seed"_a = optional<unsigned>{});
}
