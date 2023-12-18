#include "common.h"

// BoB robotics includes
#include "navigation/image_database.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// Third-party includes
#include "range/v3/view.hpp"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <stdexcept>

namespace {

// For optional<>
using namespace std::experimental;

namespace py = pybind11;
using namespace py::literals;
using namespace BoBRobotics::Navigation;
using namespace units::angle;

using PerfectMemoryType = PerfectMemoryRotater<>;
using InfoMaxType = InfoMaxRotater<>;

py::module pandas = py::module::import("pandas");
auto dataFrame = pandas.attr("DataFrame");
py::function dictsToDataFrame = dataFrame.attr("from_records");

template<class T>
class PyAlgoWrapperBase
{
public:
    template<class... Ts>
    PyAlgoWrapperBase(Ts&&... args)
      : m_Algo(std::forward<Ts>(args)...)
    {}

    const T &getAlgo() const { return m_Algo; }

    py::object test(const ImageSet &imageSet) const
    {
        // Return data as a numpy array for convenience
        py::array_t<float> result{ imageSet.size() };

        // Invoke func for each input image and put the results in result
        ranges::transform(imageSet.toRange(),
                          result.mutable_data(),
                          [&](const cv::Mat &image) {
                              // Check for Ctrl+C etc.
                              checkPythonErrors();

                              return m_Algo.test(image);
                          });

        if (imageSet.singleImage) {
            return (*result.begin()).cast<py::object>();
        }
        return result;
    }

    void train(const ImageSet &imageSet)
    {
        // For some reason using ranges::for_each here yields a crash
        const auto rng = imageSet.toRange();
        for (auto it = rng.begin(); it != rng.end(); ++it) {
            m_Algo.train(*it);

            // Check for Ctrl+C etc.
            checkPythonErrors();
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

protected:
    T m_Algo;

    template<size_t NumRetVals>
    py::object doRIDFInternal(const ImageSet &imageSet,
                              const std::array<const char *, NumRetVals> &labels,
                              size_t step) const
    {
        py::list result;

        // For some reason using ranges::for_each here yields a crash
        auto rng = imageSet.toRange();
        for (auto it = rng.begin(); it != rng.end(); ++it) {
            const cv::Mat image = *it;
            auto data = m_Algo.getHeading(image, {}, step);

            // Use labels to give returned values meaningful names
            py::dict dict;
            assignToDict<0>(dict, labels, data);
            result.append(std::move(dict));

            // Check for Ctrl+C etc.
            checkPythonErrors();
        }

        // No input DataFrame given
        if (!imageSet.dataFrame) {
            return dictsToDataFrame(std::move(result));
        }

        // Use the indexes from the input DataFrame
        auto dfOut = dictsToDataFrame(std::move(result),
                                      "index"_a = imageSet.getColumn("database_idx"));

        /*
         * If the test images are tagged with world-centric headings, use
         * these values to calculate the real headings, in addition to the
         * change in headings that we've already computed.
         */
        if (py::hasattr(*imageSet.dataFrame, "heading")) {
            dfOut["estimated_heading"] = dfOut["estimated_dheading"] +
                                         (*imageSet.dataFrame)["heading"];
        }

        // Concatenate with input DataFrame for convenience
        return dfOut.attr("join")(*imageSet.dataFrame);
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
    PyAlgoWrapper(const cv::Size &unwrapRes)
      : PyAlgoWrapperBase<PerfectMemoryType>(unwrapRes)
    {}

    // Invoked when unpickling
    PyAlgoWrapper(const cv::Size &unwrapRes, const ImageSet &imageSet,
                  std::vector<size_t> indexes)
      : PyAlgoWrapperBase<PerfectMemoryType>(unwrapRes)
      , m_Indexes(std::move(indexes))
    {
        PyAlgoWrapperBase<PerfectMemoryType>::train(imageSet);
    }

    void train(const ImageSet &imageSet)
    {
        // Keep track of the original database indexes of the input data, if provided
        if (imageSet.dataFrame) {
            BOB_ASSERT(m_Indexes.size() == m_Algo.getNumSnapshots());
            ranges::copy(toRange<size_t>(imageSet.getColumn("database_idx")),
                         ranges::back_inserter(m_Indexes));
        } else {
            BOB_ASSERT(m_Indexes.empty());
        }

        PyAlgoWrapperBase<PerfectMemoryType>::train(imageSet);
    }

    py::object doRIDF(const ImageSet &imageSet, size_t step) const
    {
        static constexpr std::array<const char *, 4> labels{ "estimated_dheading", "best_snap", "minval", "differences" };
        auto df = doRIDFInternal(imageSet, labels, step);

        auto bestSnap = getColumn(df, "best_snap");

        {
            // Put the RIDF for the best-matching snapshot into its own column
            std::vector<py::array_t<float>> bestRIDF;
            ranges::transform(toRange<py::array_t<float>>(getColumn(df, "differences")),
                              toRange<size_t>(bestSnap),
                              ranges::back_inserter(bestRIDF),
                              [&](const auto &diffs, const auto &bestSnap) {
                                  const size_t cols = diffs.shape(1);
                                  const size_t rows = diffs.shape(0);

                                  // Extract the row corresponding to the best-matching snap
                                  const Eigen::Map<const Eigen::ArrayXXf> diffsMat(diffs.data(), rows, cols);
                                  py::array_t<float> ridf{ static_cast<py::ssize_t>(cols) };
                                  auto *outPtr = ridf.mutable_data();
                                  for (size_t i = 0; i < cols; i++) {
                                      outPtr[i] = diffsMat(bestSnap, i);
                                  }
                                  return ridf;
                              });
            df["ridf"] = std::move(bestRIDF);
        }

        if (!m_Indexes.empty()) {
            // Also log the index of the best-matching snapshot in the training database
            BOB_ASSERT(m_Indexes.size() == m_Algo.getNumSnapshots());
            py::array_t<int> bestSnapIdx(imageSet.size());
            ranges::transform(toRange<size_t>(bestSnap),
                              bestSnapIdx.mutable_data(),
                              [&](size_t snap) {
                                  return m_Indexes[snap];
                              });
            df["best_snap_idx"] = std::move(bestSnapIdx);
        }

        /*
         * If the user gives a single image as input, make the output data
         * scalar too.
         */
        return imageSet.singleImage ? df.attr("squeeze")() : df;
    }

    const auto &getIndexes() const { return m_Indexes; }

private:
    std::vector<size_t> m_Indexes;
};

template<>
class PyAlgoWrapper<InfoMaxType>
  : public PyAlgoWrapperBase<InfoMaxType>
{
public:
    PyAlgoWrapper(const cv::Size &size, float learningRate,
                  Normalisation normalisation,
                  const optional<unsigned> &seed, optional<Eigen::MatrixXf> weights)
      : PyAlgoWrapperBase<InfoMaxType>(size, learningRate, normalisation, createWeights(size, seed, std::move(weights)))
    {}

    const auto &getWeights() const
    {
        return m_Algo.getWeights();
    }

    auto doRIDF(const ImageSet &imageSet, size_t step) const
    {
        static constexpr std::array<const char *, 3> labels{ "estimated_dheading", "minval", "ridf" };
        auto df = doRIDFInternal(imageSet, labels, step);

        /*
         * If the user gives a single image as input, make the output data
         * scalar too.
         */
        return imageSet.singleImage ? df.attr("squeeze")() : df;
    }

    static Eigen::MatrixXf
    createWeights(const cv::Size &size, const optional<unsigned> &seed,
                  optional<Eigen::MatrixXf> weights)
    {
        if (weights) {
            // Can't specify initial weights and a random seed
            BOB_ASSERT(!seed.has_value());

            return std::move(weights.value());
        }

        return generateInitialWeights(size, nullopt, seed).first;
    }

    static std::pair<Eigen::MatrixXf, unsigned>
    generateInitialWeights(const cv::Size &size,
                           const optional<int> &numHidden = nullopt,
                           const optional<unsigned> &seedArg = nullopt)
    {
        const auto seed = seedArg ? seedArg.value() : std::random_device()();
        const auto numInput = size.width * size.height;
        auto weights = InfoMax<>::generateInitialWeights(numInput,
                                                         numHidden.value_or(numInput),
                                                         seed);
        return std::make_pair(weights, seed);
    }
};

template<class Algo, class... Ts>
auto createAlgo(const optional<cv::Size> &size,
                const optional<ImageSet> &trainImages, Ts&&... otherArgs)
{
    if (!size && (!trainImages || trainImages->size() == 0)) {
        throw std::runtime_error("Must supply either image size or some training images");
    }

    /*
     * If size is given explicitly, use that, otherwise divine it from the size
     * of the first training image.
     */
    cv::Size algoSize;
    if (size) {
        algoSize = *size;
    } else {
        const auto firstSnap = py::reinterpret_borrow<py::array>(*trainImages->images.begin());
        algoSize.height = firstSnap.shape(0);
        algoSize.width = firstSnap.shape(1);
    }

    PyAlgoWrapper<Algo> algo(algoSize, std::forward<Ts>(otherArgs)...);
    if (trainImages) {
        algo.train(*trainImages);
    }

    return algo;
}

template<class Algo>
auto
addAlgo(py::handle scope, const char *name)
{
    using T = PyAlgoWrapper<Algo>;
    return py::class_<T>(scope, name)
            .def("ridf", &T::doRIDF, "image_set"_a, "step"_a = 1)
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
            .def(py::init([](const optional<cv::Size> &size, const optional<ImageSet> &trainImages) {
                     return createAlgo<PerfectMemoryType>(size, trainImages);
                 }),
                 "size"_a = nullopt,
                 "train_images"_a = nullopt)
            .def(py::pickle(
                    [](const PyAlgoWrapper<PerfectMemoryType> &wrapper) {
                        const auto &pm = wrapper.getAlgo();

                        /*
                         * NB: This doesn't handle image masks, but we currently
                         * don't support this with the Python module anyway.
                         */
                        py::list snapshots;
                        for (size_t i = 0; i < pm.getNumSnapshots(); i++) {
                            snapshots.append(pm.getSnapshot(i));
                        }

                        return py::make_tuple(
                                pm.getUnwrapResolution(),
                                std::move(snapshots),
                                wrapper.getIndexes());
                    },
                    [](const py::tuple &state) {
                        return PyAlgoWrapper<PerfectMemoryType>{
                            state[0].cast<cv::Size>(),
                            ImageSet{ state[1] },
                            state[2].cast<std::vector<size_t>>()
                        };
                    }));
    addAlgo<InfoMaxType>(m, "InfoMax")
            .def(py::init([](const optional<cv::Size> &size,
                             const optional<ImageSet> &trainImages,
                             float learningRate,
                             Normalisation normalisation,
                             const optional<unsigned> &seed,
                             optional<Eigen::MatrixXf> weights) {
                     return createAlgo<InfoMaxType>(size, trainImages, learningRate, normalisation, seed, std::move(weights));
                 }),
                 "size"_a = nullopt,
                 "train_images"_a = nullopt,
                 "learning_rate"_a = InfoMaxType::DefaultLearningRate,
                 "normalisation"_a = Normalisation::None,
                 "seed"_a = nullopt,
                 "weights"_a = nullopt)
            .def(py::pickle(
                    [](const PyAlgoWrapper<InfoMaxType> &wrapper) {
                        const auto &infomax = wrapper.getAlgo();
                        return py::make_tuple(
                                infomax.getUnwrapResolution(),
                                infomax.getLearningRate(),
                                infomax.getNormalisationMethod(),
                                infomax.getWeights());
                    },
                    [](const py::tuple &state) {
                        return PyAlgoWrapper<InfoMaxType>{
                            state[0].cast<cv::Size>(),
                            state[1].cast<float>(),
                            state[3].cast<Normalisation>(),
                            nullopt,
                            state[4].cast<InfoMaxType::MatrixType>()
                        };
                    }))
            .def("get_weights", &PyAlgoWrapper<InfoMaxType>::getWeights)
            .def_static("generate_initial_weights", &PyAlgoWrapper<InfoMaxType>::generateInitialWeights, "size"_a, "num_hidden"_a = ::optional<int>{}, "seed"_a = ::optional<unsigned>{})
            .def_property_readonly_static("DEFAULT_LEARNING_RATE", [](const py::object &) { return InfoMaxType::DefaultLearningRate; });
}
} // Navigation
} // BoBRobotics
