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

    auto test(const py::object &imageSet) const
    {
        return runOneOrMany(imageSet,
                            [this](const cv::Mat &image) { return m_Algo.test(image); });
    }

    void train(py::object imageSet)
    {
        py::object images;
        if (py::hasattr(imageSet, "iloc")) {
            images = imageSet["image"].attr("to_list")();
        } else {
            images = std::move(imageSet);
        }

        const py::array npArray = atLeast2d(images);
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
    template<size_t Index, size_t NumRetVals, class TupleType, std::enable_if_t<Index<NumRetVals, int> = 0>
    static void assignToDict(py::dict &dict, const std::array<const char *, NumRetVals> &labels, TupleType &data)
    {
        dict[labels[Index]] = std::move(std::get<Index>(data));
        assignToDict<Index + 1>(dict, labels, data);
    }

    template<size_t Index, size_t NumRetVals, class TupleType, std::enable_if_t<Index == NumRetVals, int> = 0>
    static void assignToDict(const py::dict &,
                             const std::array<const char *, NumRetVals> &,
                             const TupleType &)
    {
    }

    template<class Range, size_t NumRetVals>
    py::object doRIDFInternal(const Range &range,
                              const std::experimental::optional<py::object> &dfIn,
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

        // No input DataFrame given
        if (!dfIn) {
            return dictsToDataFrame(std::move(result));
        }

        // Use the indexes from the input DataFrame
        auto dfOut = dictsToDataFrame(std::move(result), "index"_a = dfIn->attr("database_idx"));

        /*
         * If the test images are tagged with world-centric headings, use
         * these values to calculate the real headings, in addition to the
         * change in headings that we've already computed.
         */
        if (py::hasattr(*dfIn, "heading")) {
            dfOut["estimated_heading"] = dfOut["estimated_dheading"].attr("to_numpy")() +
                                         (*dfIn)["heading"].attr("to_numpy")();
        }

        // Concatenate with input DataFrame for convenience
        return dfOut.attr("join")(*dfIn);
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
            ranges::transform(toRange<cv::Mat>(npArray), result.mutable_data(), [&](const cv::Mat &image) {
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

    template<size_t NumRetVals>
    auto doRIDFInternal(py::object imageSet,
                        const std::array<const char *, NumRetVals> &labels) const
    {
        // If imageSet is a DataFrame, extract the images and index columns
        py::object images;
        std::experimental::optional<py::object> dataFrameIn;
        if (py::hasattr(imageSet, "iloc")) {
            images = imageSet.attr("image").attr("to_list")();
            dataFrameIn.emplace(std::move(imageSet));
        } else {
            images = std::move(imageSet);
        }

        const py::array npArray = atLeast2d(images);
        switch (npArray.ndim()) {
        case 2: { // Single image
            // Single-element range
            auto rng = ranges::views::single(npArray.cast<cv::Mat>());

            // We call squeeze() to turn DataFrame's array members into scalars
            return doRIDFInternal(std::move(rng), dataFrameIn, labels).attr("squeeze")();
        } break;
        case 3: // Multiple images
            return doRIDFInternal(toRange<cv::Mat>(images), dataFrameIn, labels);
        default:
            throw std::invalid_argument("Wrong number of dimensions");
        }
    }
};

template<class T>
class PyAlgoWrapper
{};

template<>
class PyAlgoWrapper<PerfectMemoryType>
  : public PyAlgoWrapperBase<PerfectMemoryType>
{
public:
    template<class... Ts>
    PyAlgoWrapper(Ts &&...args)
      : PyAlgoWrapperBase<PerfectMemoryType>(std::forward<Ts>(args)...)
    {}

    void train(py::object imageSet)
    {
        // Keep track of the original database indexes of the input data, if provided
        if (py::hasattr(imageSet, "iloc")) {
            BOB_ASSERT(m_Indexes.size() == m_Algo.getNumSnapshots());
            ranges::copy(toRange<size_t>(imageSet.attr("database_idx")),
                         ranges::back_inserter(m_Indexes));
        } else {
            BOB_ASSERT(m_Indexes.empty());
        }

        PyAlgoWrapperBase<PerfectMemoryType>::train(std::move(imageSet));
    }

    py::object doRIDF(py::object imageSet) const
    {
        static constexpr std::array<const char *, 4> labels{ "estimated_dheading", "best_snap", "minval", "differences" };
        auto df = doRIDFInternal(std::move(imageSet), labels);

        if (m_Indexes.empty()) {
            // ...then snapshots were loaded without giving indexes
            return df;
        }

        const auto dfLen = static_cast<py::ssize_t>(py::len(df));

        // Also log the index of the best-matching snapshot in the training database
        BOB_ASSERT(m_Indexes.size() == m_Algo.getNumSnapshots());
        py::array_t<int> bestSnapIdx(dfLen);
        ranges::transform(toRange<size_t>(df["best_snap"]),
                          bestSnapIdx.mutable_data(),
                          [&](size_t snap) {
                              return m_Indexes[snap];
                          });
        df["best_snap_idx"] = std::move(bestSnapIdx);

        // Also put the RIDFs for best-matching snaps into their own column, for convenience
        py::list bestRIDF;
        ranges::for_each(ranges::views::zip(toRange<py::array_t<float>>(df["differences"]),
                                            toRange<size_t>(df["best_snap"])),
                         [&](const auto &data) {
                             const size_t cols = data.first.shape(1);

                             // Extract the row corresponding to the best-matching snap
                             py::array_t<float> ridf(cols, &data.first.data()[data.second * cols], data.first);
                             bestRIDF.append(std::move(ridf));
                         });
        df["ridf"] = std::move(bestRIDF);

        return df;
    }

private:
    std::vector<size_t> m_Indexes;
};

template<>
class PyAlgoWrapper<InfoMaxType>
  : public PyAlgoWrapperBase<InfoMaxType>
{
public:
    PyAlgoWrapper(const cv::Size &size, float learningRate, float tanhScalingFactor, Normalisation normalisation, Eigen::MatrixXf weights)
      : PyAlgoWrapperBase<InfoMaxType>(size, learningRate, tanhScalingFactor, normalisation, std::move(weights))
    {}

    PyAlgoWrapper(const cv::Size &size, float learningRate, float tanhScalingFactor, Normalisation normalisation)
      : PyAlgoWrapper(size, learningRate, tanhScalingFactor, normalisation, generateInitialWeights(size).first)
    {}

    const auto &getWeights() const
    {
        return m_Algo.getWeights();
    }

    auto doRIDF(py::object imageSet) const
    {
        static constexpr std::array<const char *, 3> labels{ "estimated_dheading", "minval", "ridf" };
        return doRIDFInternal(std::move(imageSet), labels);
    }

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
            .def("ridf", &T::doRIDF)
            .def("test", &T::test)
            .def("train", &T::train);
}
} // anonymous namespace

namespace BoBRobotics {
namespace Navigation {
void
addAlgorithmClasses(py::module &m)
{
    py::enum_<Normalisation>(m, "Normalisation")
            .value("none", Normalisation::None)
            .value("zscore", Normalisation::ZScore);

    // Add various algorithms as Python classes
    addAlgo<PerfectMemoryType>(m, "PerfectMemory")
            .def(py::init<const cv::Size &>());
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
            .def("get_weights", &PyAlgoWrapper<InfoMaxType>::getWeights)
            .def_static("generate_initial_weights", &PyAlgoWrapper<InfoMaxType>::generateInitialWeights, "size"_a, "num_hidden"_a = ::optional<int>{}, "seed"_a = ::optional<unsigned>{})
            .def_property_readonly_static("DEFAULT_LEARNING_RATE", [](const py::object &) { return InfoMaxType::DefaultLearningRate; })
            .def_property_readonly_static("DEFAULT_TANH_SCALING_FACTOR", [](const py::object &) { return InfoMaxType::DefaultTanhScalingFactor; });
}
} // Navigation
} // BoBRobotics
