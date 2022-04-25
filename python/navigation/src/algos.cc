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

py::module numpy = py::module::import("numpy");
py::function atLeast2d = numpy.attr("atleast_2d");

py::module pandas = py::module::import("pandas");
auto dataFrame = pandas.attr("DataFrame");
py::function dictsToDataFrame = dataFrame.attr("from_records");

std::pair<py::object, bool>
getColumnAsSequence(const py::object &df, const char *name)
{
    auto column = df[name];

    // Regular column with multiple values
    if (py::hasattr(column, "to_list")) {
        return std::make_pair(column, true);
    }

    // Scalar value: wrap in tuple
    return std::make_pair(py::make_tuple(std::move(column)), false);
}

std::pair<py::array, std::experimental::optional<py::object>>
parseImageArgument(py::object imageSet)
{
    py::object images;
    std::experimental::optional<py::object> dataFrame;
    if (py::hasattr(imageSet, "iloc")) {
        // ...then it's a DataFrame/Series
        images = imageSet["image"];

        // If it's a column with multiple values
        if (py::hasattr(images, "to_list")) {
            images = images.attr("to_list")();
        }

        dataFrame.emplace(std::move(imageSet));
    } else {
        images = std::move(imageSet);
    }

    py::array images2d = atLeast2d(std::move(images));
    BOB_ASSERT(images2d.ndim() <= 3);
    return std::make_pair(std::move(images2d), std::move(dataFrame));
}

template<class T>
class PyAlgoWrapperBase
{
public:
    template<class... Ts>
    PyAlgoWrapperBase(Ts&&... args)
      : m_Algo(std::forward<Ts>(args)...)
    {}

    const T &getAlgo() const { return m_Algo; }

    py::object test(py::object imageSet) const
    {
        const py::array npArray = parseImageArgument(std::move(imageSet)).first;
        if (npArray.ndim() == 2) {
            return py::cast(m_Algo.test(npArray.cast<cv::Mat>()));
        }

        // Return data as a numpy array for convenience
        const py::ssize_t len = py::len(npArray);
        py::array_t<float> result{ len };

        // Invoke func for each input image and put the results in result
        ranges::transform(toRange<cv::Mat>(npArray), result.mutable_data(), [&](const cv::Mat &image) {
            // Check for Ctrl+C etc.
            BOB_ASSERT(!PyErr_CheckSignals());

            return m_Algo.test(image);
        });

        return result;
    }

    void train(py::object imageSet)
    {
        const py::array npArray = parseImageArgument(std::move(imageSet)).first;
        if (npArray.ndim() == 2) {
            m_Algo.train(npArray.cast<cv::Mat>());
            return;
        }

        auto train = [this](const cv::Mat &image) {
            m_Algo.train(image);

            // Check for Ctrl+C etc.
            BOB_ASSERT(!PyErr_CheckSignals());
        };
        ranges::for_each(toRange<cv::Mat>(npArray), train);
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
        auto dfOut = dictsToDataFrame(std::move(result), "index"_a = getColumnAsSequence(*dfIn, "database_idx").first);

        /*
         * If the test images are tagged with world-centric headings, use
         * these values to calculate the real headings, in addition to the
         * change in headings that we've already computed.
         */
        if (py::hasattr(*dfIn, "heading")) {
            dfOut["estimated_heading"] = dfOut["estimated_dheading"] +
                                         (*dfIn)["heading"];
        }

        // Concatenate with input DataFrame for convenience
        return dfOut.attr("join")(*dfIn);
    }

protected:
    T m_Algo;

    template<size_t NumRetVals>
    auto doRIDFInternal(py::object imageSet,
                        const std::array<const char *, NumRetVals> &labels) const
    {
        py::array npArray;
        std::experimental::optional<py::object> dataFrameIn;
        std::tie(npArray, dataFrameIn) = parseImageArgument(std::move(imageSet));

        if (npArray.ndim() == 2) {
            // Single-element range
            auto rng = ranges::views::single(npArray.cast<cv::Mat>());

            // We call squeeze() to turn DataFrame's array members into scalars
            return doRIDFInternal(std::move(rng), dataFrameIn, labels).attr("squeeze")();
        }

        return doRIDFInternal(toRange<cv::Mat>(npArray), dataFrameIn, labels);
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
            ranges::copy(toRange<size_t>(getColumnAsSequence(imageSet, "database_idx").first),
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

        // Also log the index of the best-matching snapshot in the training database
        BOB_ASSERT(m_Indexes.size() == m_Algo.getNumSnapshots());
        py::object bestSnap;
        bool isSequence;
        std::tie(bestSnap, isSequence) = getColumnAsSequence(df, "best_snap");
        py::array_t<int> bestSnapIdx(isSequence ? py::len(df) : 1);
        ranges::transform(toRange<size_t>(bestSnap),
                          bestSnapIdx.mutable_data(),
                          [&](size_t snap) {
                              return m_Indexes[snap];
                          });

        // Put the RIDF for the best-matching snapshot into its own column
        std::vector<py::array_t<float>> bestRIDF;
        ranges::transform(toRange<py::array_t<float>>(getColumnAsSequence(df, "differences").first),
                          toRange<size_t>(bestSnap),
                          ranges::back_inserter(bestRIDF),
                          [&](const auto &diffs, const auto &bestSnap) {
                              const size_t cols = diffs.shape(1);

                              // Extract the row corresponding to the best-matching snap
                              const float *ptr = &diffs.data()[bestSnap * cols];
                              return py::array_t<float>(cols, ptr, diffs);
                          });

        if (isSequence) {
            df["best_snap_idx"] = std::move(bestSnapIdx);
            df["ridf"] = std::move(bestRIDF);
        } else {
            df["best_snap_idx"] = *bestSnapIdx.begin();
            df["ridf"] = bestRIDF[0];
        }

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

    PyAlgoWrapper(const cv::Size &size, float learningRate, const std::experimental::optional<unsigned> &seed, float tanhScalingFactor, Normalisation normalisation)
      : PyAlgoWrapper(size, learningRate, tanhScalingFactor, normalisation, generateInitialWeights(size, std::experimental::nullopt, seed).first)
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
            .def(py::init<const cv::Size &, float, const std::experimental::optional<unsigned> &, float, Normalisation>(),
                 "size"_a,
                 "learning_rate"_a = InfoMaxType::DefaultLearningRate,
                 "seed"_a = std::experimental::nullopt,
                 "tanh_scaling_factor"_a = InfoMaxType::DefaultTanhScalingFactor,
                 "normalisation"_a = Normalisation::None)
            .def(py::init<const cv::Size &, float, float, Normalisation, Eigen::MatrixXf>(),
                 "size"_a,
                 "learning_rate"_a = InfoMaxType::DefaultLearningRate,
                 "tanh_scaling_factor"_a = InfoMaxType::DefaultTanhScalingFactor,
                 "normalisation"_a = Normalisation::None,
                 "weights"_a)
            .def(py::pickle(
                    [](const PyAlgoWrapper<InfoMaxType> &wrapper) {
                        const auto &infomax = wrapper.getAlgo();
                        return py::make_tuple(
                                infomax.getUnwrapResolution(),
                                infomax.getLearningRate(),
                                infomax.getTanhScalingFactor(),
                                infomax.getNormalisationMethod(),
                                infomax.getWeights());
                    },
                    [](const py::tuple &state) {
                        return PyAlgoWrapper<InfoMaxType>{
                            state[0].cast<cv::Size>(),
                            state[1].cast<float>(),
                            state[2].cast<float>(),
                            state[3].cast<Normalisation>(),
                            state[4].cast<InfoMaxType::MatrixType>()
                        };
                    }))
            .def("get_weights", &PyAlgoWrapper<InfoMaxType>::getWeights)
            .def_static("generate_initial_weights", &PyAlgoWrapper<InfoMaxType>::generateInitialWeights, "size"_a, "num_hidden"_a = ::optional<int>{}, "seed"_a = ::optional<unsigned>{})
            .def_property_readonly_static("DEFAULT_LEARNING_RATE", [](const py::object &) { return InfoMaxType::DefaultLearningRate; })
            .def_property_readonly_static("DEFAULT_TANH_SCALING_FACTOR", [](const py::object &) { return InfoMaxType::DefaultTanhScalingFactor; });
}
} // Navigation
} // BoBRobotics
