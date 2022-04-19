#include "common.h"

// BoB robotics includes
#include "navigation/image_database.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// Third-party includes
#include "range/v3/view.hpp"

// Standard C++ includes
#include <stdexcept>

namespace {

template<class T>
using optional = std::experimental::optional<T>;

namespace py = pybind11;
using namespace py::literals;
using namespace BoBRobotics::Navigation;
using namespace units::angle;

using PerfectMemoryType = PerfectMemoryRotater<>;
using InfoMaxType = InfoMaxRotater<>;

namespace {
py::module numpy = py::module::import("numpy");
py::function atLeast2d = numpy.attr("atleast_2d");

py::module pandas = py::module::import("pandas");
auto dataFrame = pandas.attr("DataFrame");
py::function dictsToDataFrame = dataFrame.attr("from_records");
} // anonymous namespace

template<class T>
class PyAlgoWrapperBase
{
public:
    template<class... Ts>
    PyAlgoWrapperBase(Ts&&... args)
      : m_Algo(std::forward<Ts>(args)...)
    {}

    auto getHeading(const py::object &imageSet) const
    {
        return runOneOrMany(imageSet,
                            [this](const cv::Mat &image) { return std::get<0>(m_Algo.getHeading(image)); });
    }

    template<size_t NumRetVals>
    auto getRIDFData(const py::object &imageSet,
                     const std::array<const char *, NumRetVals> &labels) const
    {
        const py::array npArray = atLeast2d(imageSet);
        switch (npArray.ndim()) {
        case 2: { // Single image
            // Single-element range
            const auto rng = ranges::views::single(npArray.cast<cv::Mat>());

            // We call squeeze() to turn DataFrame's array members into scalars
            return getHeadingDataMany(rng, labels).attr("squeeze")();
        } break;
        case 3: // Multiple images
            return getHeadingDataMany(toRange<cv::Mat>(imageSet), labels);
        default:
            throw std::invalid_argument("Wrong number of dimensions");
        }
    }

    auto test(const py::object &imageSet) const
    {
        return runOneOrMany(imageSet,
                            [this](const cv::Mat &image) { return m_Algo.test(image); });
    }

    void train(const py::object &imageSet)
    {
        const py::array npArray = atLeast2d(imageSet);
        switch (npArray.ndim()) {
        case 2:
            m_Algo.train(npArray.cast<cv::Mat>());
            break;
        case 3: {
            auto train = [this](const cv::Mat &image) {
                m_Algo.train(image);

                // Check for Ctrl+C etc.
                BOB_ASSERT(!PyErr_CheckSignals());
            };
            ranges::for_each(toRange<cv::Mat>(npArray), train);
        } break;
        default:
            throw std::invalid_argument("Wrong number of dimensions");
        }
    }

private:
    template<size_t Index, size_t NumRetVals, class TupleType,
             std::enable_if_t<Index < NumRetVals, int> = 0>
    static void assignToDict(py::dict &dict,
                             const std::array<const char *, NumRetVals> &labels,
                             TupleType &data)
    {
        dict[labels[Index]] = std::move(std::get<Index>(data));
        assignToDict<Index + 1>(dict, labels, data);
    }

    template<size_t Index, size_t NumRetVals, class TupleType,
             std::enable_if_t<Index == NumRetVals, int> = 0>
    static void assignToDict(const py::dict &,
                             const std::array<const char *, NumRetVals> &,
                             const TupleType &)
    {
    }

    template<class Range, size_t NumRetVals>
    py::object getHeadingDataMany(const Range &range,
                                  const std::array<const char *, NumRetVals> &labels) const
    {
        py::list result;

        ranges::for_each(range, [&](const cv::Mat &image) {
            auto data = m_Algo.getHeading(image);

            // Use labels to give returned values meaningful names
            py::dict dict;
            assignToDict<0>(dict, labels, data);
            result.append(std::move(dict));

            // Check for Ctrl+C etc.
            BOB_ASSERT(!PyErr_CheckSignals());
        });

        // Convert returned data to a pandas DataFrame
        return dictsToDataFrame(std::move(result));
    }

    static float toFloat(const radian_t &val)
    {
        return val.value();
    }

    static float toFloat(float val)
    {
        return val;
    }

    template<class Func>
    static py::object runOneOrMany(const py::object &imageSet, const Func &func)
    {
        const py::array npArray = atLeast2d(imageSet);
        switch (npArray.ndim()) {
        case 2:
            return py::cast(func(npArray.cast<cv::Mat>()));
        case 3: {
            // Return data as a numpy array for convenience
            const py::ssize_t len = py::len(npArray);
            py::array_t<float> result{ len };

            // Invoke func for each input image and put the results in result
            ranges::transform(toRange<cv::Mat>(npArray), result.mutable_data(),
                [&](const cv::Mat &image) {
                    // Check for Ctrl+C etc.
                    BOB_ASSERT(!PyErr_CheckSignals());

                    return toFloat(func(image));
                });

            return result;
        }
        default:
            throw std::invalid_argument("Wrong number of dimensions");
        }
    }

protected:
    T m_Algo;
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
                  float tanhScalingFactor, Normalisation normalisation,
                  Eigen::MatrixXf weights)
      : PyAlgoWrapperBase<InfoMaxType>(size, learningRate, tanhScalingFactor,
                                       normalisation, std::move(weights))
    {}

    PyAlgoWrapper(const cv::Size &size, float learningRate,
                  float tanhScalingFactor, Normalisation normalisation)
      : PyAlgoWrapper(size, learningRate, tanhScalingFactor, normalisation,
                      generateInitialWeights(size).first)
    {}

    const auto &getWeights() const { return m_Algo.getWeights(); }

    static std::pair<Eigen::MatrixXf, unsigned>
    generateInitialWeights(const cv::Size &size,
                           const optional<int> &numHidden = std::experimental::nullopt,
                           const optional<unsigned> &seedArg = std::experimental::nullopt)
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
            .def("test", &T::test)
            .def("train", &T::train);
}
} // anonymous namespace

namespace BoBRobotics {
namespace Navigation {
void addAlgorithmClasses(py::module &m)
{
    py::enum_<Normalisation>(m, "Normalisation")
            .value("none", Normalisation::None)
            .value("zscore", Normalisation::ZScore);

    // Add various algorithms as Python classes
    addAlgo<PerfectMemoryType>(m, "PerfectMemory")
            .def(py::init<const cv::Size &>())
            .def("get_ridf_data", [](const PyAlgoWrapper<PerfectMemoryType> &pm, const py::object &imageSet) {
                // TODO: Also return RIDFs
                return pm.getRIDFData(imageSet, std::array<const char *, 3>{ "heading", "best_snap", "minval" });
            });
    addAlgo<InfoMaxType>(m, "InfoMax")
            .def(py::init<const cv::Size &, float, float, Normalisation>(),
                 "size"_a,
                 "learning_rate"_a = InfoMaxType::DefaultLearningRate,
                 "tanh_scaling_factor"_a = InfoMaxType::DefaultTanhScalingFactor,
                 "normalisation"_a = Normalisation::None)
            .def(py::init<const cv::Size &, float, float, Normalisation, Eigen::MatrixXf>(),
                 "size"_a,
                 "learning_rate"_a = InfoMaxType::DefaultLearningRate,
                 "tanh_scaling_factor"_a = InfoMaxType::DefaultTanhScalingFactor,
                 "normalisation"_a = Normalisation::None,
                 "weights"_a)
            .def("get_ridf_data", [](const PyAlgoWrapper<InfoMaxType> &infomax, const py::object &imageSet) {
                return infomax.getRIDFData(imageSet, std::array<const char *, 2>{ "heading", "minval" });
            })
            .def("get_weights", &PyAlgoWrapper<InfoMaxType>::getWeights)
            .def_static("generate_initial_weights", &PyAlgoWrapper<InfoMaxType>::generateInitialWeights, "size"_a, "num_hidden"_a = ::optional<int>{}, "seed"_a = ::optional<unsigned>{})
            .def_property_readonly_static("DEFAULT_LEARNING_RATE", [](const py::object &) { return InfoMaxType::DefaultLearningRate; })
            .def_property_readonly_static("DEFAULT_TANH_SCALING_FACTOR", [](const py::object &) { return InfoMaxType::DefaultTanhScalingFactor; });
}
} // Navigation
} // BoBRobotics
