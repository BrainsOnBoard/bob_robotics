#include "memory.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/progress_bar.h"
#include "common/serialise_matrix.h"
#include "imgproc/mask.h"
#include "navigation/image_database.h"

// BoB robotics third party includes
#include "plog/Log.h"
#include "third_party/path.h"

// Snapshot bot includes
#include "config.h"
#include "image_input.h"

// Standard C++ includes
#include <sstream>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

namespace {
constexpr const char *CSVBestHeading = "Best heading [degrees]";
constexpr const char *CSVLowestDifference = "Lowest difference";
constexpr const char *CSVBestSnapshot = "Best snapshot index";
constexpr const char *CSVWindowStart = "Window start";
constexpr const char *CSVWindowEnd = "Window end";

std::string getWeightsFileName(const cv::Size &inputSize)
{
    std::stringstream ss;
    ss << "weights" << inputSize.width << "x" << inputSize.height << ".bin";
    return ss.str();
}
}

//------------------------------------------------------------------------
// MemoryBase
//------------------------------------------------------------------------
MemoryBase::MemoryBase()
:   m_BestHeading(0.0_deg), m_LowestDifference(std::numeric_limits<float>::infinity())
{
}
//------------------------------------------------------------------------
std::vector<std::string> MemoryBase::getCSVFieldNames() const
{
    return { CSVBestHeading, CSVLowestDifference };
}
//------------------------------------------------------------------------
void MemoryBase::setCSVFieldValues(std::unordered_map<std::string, std::string> &fields) const
{
    fields[CSVBestHeading] = std::to_string(getBestHeading().value());
    fields[CSVLowestDifference] = std::to_string(getLowestDifference());
}
//------------------------------------------------------------------------
void MemoryBase::trainRoute(const Navigation::ImageDatabase &route,
                            ImageInput &imageInput,
                            size_t testFrameSkip,
                            BackgroundExceptionCatcher *backgroundEx)
{
    const size_t numSnaps = (testFrameSkip < route.size()) ? route.size() / testFrameSkip : 1;
    std::vector<cv::Mat> snapshots(numSnaps);

    // Load and process images in parallel
    // **TODO**: Add support for static mask images
    {
        ProgressBar loadProgBar{ "Loading snapshots", numSnaps };
        route.forEachImage(
            [&](size_t i, const cv::Mat &snapshot) {
                if (backgroundEx) {
                    backgroundEx->check();
                }

                // Store snapshot in vector
                snapshots[i] = snapshot;
                loadProgBar.increment();
            },
            /*frameSkip=*/testFrameSkip,
            /*greyscale=*/false);
    }

    // Train model
    {
        ProgressBar trainProgBar{ "Training", numSnaps };
        for (const auto &snapshot : snapshots) {
            if (backgroundEx) {
                backgroundEx->check();
            }

            // Process snapshot
            // **NOTE** ImageInput classes are NOT THREAD-SAFE so this needs to be done in serial
            const auto processedSnapshot = imageInput.processSnapshot(snapshot);

            train(processedSnapshot.first, processedSnapshot.second);
            trainProgBar.increment();
        }
    }
}

//------------------------------------------------------------------------
// PerfectMemory
//------------------------------------------------------------------------
PerfectMemory::PerfectMemory(const Config&, const cv::Size &inputSize)
:   m_PM(inputSize), m_BestSnapshotIndex(std::numeric_limits<size_t>::max())
{
}
//------------------------------------------------------------------------
void PerfectMemory::test(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    // Get heading directly from Perfect Memory
    degree_t bestHeading;
    float lowestDifference;
    std::tie(bestHeading, m_BestSnapshotIndex, lowestDifference, std::ignore) = getPM().getHeading(snapshot, mask);

    // Set best heading and vector length
    setBestHeading(bestHeading);
    setLowestDifference(lowestDifference);
}
//------------------------------------------------------------------------
void PerfectMemory::train(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    getPM().train(snapshot, mask);
}
//------------------------------------------------------------------------
std::vector<std::string> PerfectMemory::getCSVFieldNames() const
{
    // Superclass
    auto fieldNames = MemoryBase::getCSVFieldNames();

    fieldNames.emplace_back(CSVBestSnapshot);
    return fieldNames;
}
//------------------------------------------------------------------------
void PerfectMemory::setCSVFieldValues(std::unordered_map<std::string, std::string> &fields) const
{
    // Superclass
    MemoryBase::setCSVFieldValues(fields);

    fields[CSVBestSnapshot] = std::to_string(getBestSnapshotIndex());
}

//------------------------------------------------------------------------
// PerfectMemoryConstrained
//------------------------------------------------------------------------
PerfectMemoryConstrained::PerfectMemoryConstrained(const Config &config, const cv::Size &inputSize)
:   PerfectMemory(config, inputSize), m_ImageWidth(inputSize.width),
    m_NumScanColumns((size_t)std::round(turn_t(config.getMaxSnapshotRotateAngle()).value() * (double)inputSize.width))
{
}
//------------------------------------------------------------------------
void PerfectMemoryConstrained::test(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    // Get best heading from left side of scan
    degree_t leftBestHeading;
    float leftLowestDifference;
    size_t leftBestSnapshot;
    std::tie(leftBestHeading, leftBestSnapshot, leftLowestDifference, std::ignore) = getPM().getHeading(
        snapshot, mask, 1, 0, m_NumScanColumns);

    // Get best heading from right side of scan
    degree_t rightBestHeading;
    float rightLowestDifference;
    size_t rightBestSnapshot;
    std::tie(rightBestHeading, rightBestSnapshot, rightLowestDifference, std::ignore) = getPM().getHeading(
        snapshot, mask, 1, m_ImageWidth - m_NumScanColumns, m_ImageWidth);

    // Get lowest difference and best heading from across scans
    setLowestDifference(std::min(leftLowestDifference, rightLowestDifference) / 255.0f);
    setBestHeading((leftLowestDifference < rightLowestDifference) ? leftBestHeading : rightBestHeading);
    setBestSnapshotIndex((leftLowestDifference < rightLowestDifference) ? leftBestSnapshot : rightBestSnapshot);
}

//------------------------------------------------------------------------
// PerfectMemoryConstrainedDynamicWindow
//------------------------------------------------------------------------
PerfectMemoryConstrainedDynamicWindow::PerfectMemoryConstrainedDynamicWindow(const Config &config, const cv::Size &inputSize)
:   PerfectMemory(config, inputSize), m_ImageWidth(inputSize.width),
    m_NumScanColumns((size_t)std::round(turn_t(config.getMaxSnapshotRotateAngle()).value() * (double)inputSize.width)),
    m_Window(config.getPMFwdLASize(), config.getPMFwdConfig())
{
}
//------------------------------------------------------------------------
void PerfectMemoryConstrainedDynamicWindow::test(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    // Get current window
    const auto window = m_Window.getWindow(getPM().getNumSnapshots());

    // Get best heading from left side of scan
    degree_t leftBestHeading;
    float leftLowestDifference;
    size_t leftBestSnapshot;
    std::tie(leftBestHeading, leftBestSnapshot, leftLowestDifference, std::ignore) = getPM().getHeading(
        snapshot, mask, window, 1, 0, m_NumScanColumns);

    // Get best heading from right side of scan
    degree_t rightBestHeading;
    float rightLowestDifference;
    size_t rightBestSnapshot;
    std::tie(rightBestHeading, rightBestSnapshot, rightLowestDifference, std::ignore) = getPM().getHeading(
        snapshot, mask, window, 1, m_ImageWidth - m_NumScanColumns, m_ImageWidth);

    // If best result came from left scan
    if(leftLowestDifference < rightLowestDifference) {
        setLowestDifference(leftLowestDifference / 255.0f);
        setBestHeading(leftBestHeading);
        setBestSnapshotIndex(leftBestSnapshot);

        m_Window.updateWindow(leftBestSnapshot, leftLowestDifference);
    }
    else {
        setLowestDifference(rightLowestDifference / 255.0f);
        setBestHeading(rightBestHeading);
        setBestSnapshotIndex(rightBestSnapshot);

        m_Window.updateWindow(rightBestSnapshot, rightLowestDifference);
    }
}
//------------------------------------------------------------------------
std::vector<std::string> PerfectMemoryConstrainedDynamicWindow::getCSVFieldNames() const
{
    // Superclass
    auto fieldNames = PerfectMemory::getCSVFieldNames();

    fieldNames.emplace_back(CSVWindowStart);
    fieldNames.emplace_back(CSVWindowEnd);
    return fieldNames;
}
//------------------------------------------------------------------------
void PerfectMemoryConstrainedDynamicWindow::setCSVFieldValues(std::unordered_map<std::string, std::string> &fields) const
{
    // Superclass
    PerfectMemory::setCSVFieldValues(fields);

    const auto window = m_Window.getWindow(getPM().getNumSnapshots());
    fields[CSVWindowStart] = std::to_string(window.first);
    fields[CSVWindowEnd] = std::to_string(window.second);
}

//------------------------------------------------------------------------
// InfoMax
//------------------------------------------------------------------------
InfoMax::InfoMax(const Config &config, const cv::Size &inputSize)
:   m_InfoMax(createInfoMax(config, inputSize))
{
    LOGI << "\tUsing " << Eigen::nbThreads() << " threads";
}
//------------------------------------------------------------------------
void InfoMax::test(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    BOB_ASSERT(mask.empty());

    // Get heading directly from InfoMax
    degree_t bestHeading;
    float lowestDifference;
    std::tie(bestHeading, lowestDifference, std::ignore) = getInfoMax().getHeading(snapshot, mask);

    // Set best heading and vector length
    setBestHeading(bestHeading);
    setLowestDifference(lowestDifference);
}
//-----------------------------------------------------------------------
void InfoMax::train(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    BOB_ASSERT(mask.empty());
    getInfoMax().train(snapshot, mask);
}
//-----------------------------------------------------------------------
void InfoMax::trainRoute(const Navigation::ImageDatabase &route,
                         ImageInput &imageInput,
                         size_t testFrameSkip,
                         BackgroundExceptionCatcher* backgroundEx)
{
    // If this file exists then we've already trained the network...
    const auto weightsPath = route.getPath() / getWeightsFileName(imageInput.getOutputSize());
    if (weightsPath.exists()) {
        return;
    }

    // ...otherwise, train it now
    MemoryBase::trainRoute(route, imageInput, testFrameSkip, backgroundEx);
    saveWeights(weightsPath);
}
//-----------------------------------------------------------------------
void InfoMax::saveWeights(const filesystem::path &filename) const
{
    // Write weights to disk
    writeMatrix(filename, getInfoMax().getWeights());
}
//-----------------------------------------------------------------------
InfoMax::InfoMaxType InfoMax::createInfoMax(const Config &config, const cv::Size &inputSize)
{
    const filesystem::path weightPath = config.getOutputPath() / getWeightsFileName(inputSize);
    if (weightPath.exists()) {
        LOGI << "\tLoading weights from " << weightPath;

        const auto weights = readMatrix<InfoMaxWeightMatrixType::Scalar>(weightPath);
        return { inputSize, InfoMaxType::DefaultLearningRate,
                 Navigation::Normalisation::None, weights };
    } else {
        return { inputSize };
    }
}

//-----------------------------------------------------------------------
// InfoMaxConstrained
//-----------------------------------------------------------------------
InfoMaxConstrained::InfoMaxConstrained(const Config &config, const cv::Size &inputSize)
:   InfoMax(config, inputSize), m_ImageWidth(inputSize.width),
    m_NumScanColumns((size_t)std::round(turn_t(config.getMaxSnapshotRotateAngle()).value() * (double)inputSize.width))
{
}
//-----------------------------------------------------------------------
void InfoMaxConstrained::test(const cv::Mat &snapshot, const ImgProc::Mask &mask)
{
    BOB_ASSERT(mask.empty());
    // Get best heading from left side of scan
    degree_t leftBestHeading;
    float leftLowestDifference;
    std::tie(leftBestHeading, leftLowestDifference, std::ignore) = getInfoMax().getHeading(
        snapshot, mask, 1, 0, m_NumScanColumns);

    // Get best heading from right side of scan
    degree_t rightBestHeading;
    float rightLowestDifference;
    std::tie(rightBestHeading, rightLowestDifference, std::ignore) = getInfoMax().getHeading(
        snapshot, mask, 1, m_ImageWidth - m_NumScanColumns, m_ImageWidth);

    // Get lowest difference and best heading from across scans
    setLowestDifference(std::min(leftLowestDifference, rightLowestDifference));
    setBestHeading((leftLowestDifference < rightLowestDifference) ? leftBestHeading : rightBestHeading);
}

std::unique_ptr<MemoryBase> createMemory(const Config &config, const cv::Size &inputSize)
{
    // Create appropriate type of memory
    if(config.shouldUseInfoMax()) {
        if(config.getMaxSnapshotRotateAngle() < 180_deg) {
            LOGI << "Creating InfoMaxConstrained";
            return std::make_unique<InfoMaxConstrained>(config, inputSize);
        }
        else {
            LOGI << "Creating InfoMax";
            return std::make_unique<InfoMax>(config, inputSize);
        }
    }
    else {
        if(config.getMaxSnapshotRotateAngle() < 180_deg) {
            if(config.getPMFwdLASize() == std::numeric_limits<size_t>::max()) {
                LOGI << "Creating PerfectMemoryConstrained";
                return std::make_unique<PerfectMemoryConstrained>(config, inputSize);
            }
            else {
                LOGI << "Creating PerfectMemoryConstrainedDynamicWindow";
                return std::make_unique<PerfectMemoryConstrainedDynamicWindow>(config, inputSize);
            }
        }
        else {
            LOGI << "Creating PerfectMemory";
            return std::make_unique<PerfectMemory>(config, inputSize);
        }
    }
}
