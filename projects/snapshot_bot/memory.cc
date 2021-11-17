#define NO_HEADER_DEFINITIONS
#include "memory.h"

// BoB robotics includes
#include "common/serialise_matrix.h"
#include "imgproc/mask.h"
#include "navigation/image_database.h"

// BoB robotics third party includes
#include "plog/Log.h"
#include "third_party/path.h"

// Snapshot bot includes
#include "config.h"
#include "image_input.h"

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

constexpr const char *CSVBestHeading = "Best heading [degrees]";
constexpr const char *CSVLowestDifference = "Lowest difference";
constexpr const char *CSVBestSnapshot = "Best snapshot index";
constexpr const char *CSVWindowStart = "Window start";
constexpr const char *CSVWindowEnd = "Window end";

// Bounds used for extracting masks from ODK2 images
const cv::Scalar odk2MaskLowerBound(1, 1, 1);
const cv::Scalar odk2MaskUpperBound(255, 255, 255);

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

void MemoryBase::trainRoute(Navigation::ImageDatabase &route,
                            bool useODK2, ImageInput &imageInput)
{
    LOGI << "Loading stored snapshots...";
    std::vector<std::pair<cv::Mat, ImgProc::Mask>> snapshots(route.size());

    // Load and process images in parallel
    // **TODO**: Add support for static mask images
    route.forEachImage(
        [&](size_t i, const cv::Mat &snapshot)
        {
            std::cout << "." << std::flush;

            // If we're using ODK2, extract mask from iamge
            if(useODK2) {
                snapshots[i].second.set(snapshot, odk2MaskLowerBound, odk2MaskUpperBound);
            }

            // Process snapshot
            snapshots[i].first = imageInput.processSnapshot(snapshot);
        }, /*frameSkip=*/1, /*greyscale=*/false);
    std::cout << "\n";
    LOGI << "Loaded " << route.size() << " snapshots";

    // Train model
    LOGI << "Training model...";
    for (const auto &snapshot : snapshots) {
        std::cout << "." << std::flush;
        train(snapshot.first, snapshot.second);
    }
    std::cout << "\n";
    LOGI << "Training complete";
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
void InfoMax::trainRoute(Navigation::ImageDatabase &route, bool useODK2,
                         ImageInput &imageInput)
{
    // If this file exists then we've already trained the network...
    const auto weightsPath = route.getPath() / "weights.bin";
    if (weightsPath.exists()) {
        return;
    }

    // ...otherwise, train it now
    MemoryBase::trainRoute(route, useODK2, imageInput);
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
    const filesystem::path weightPath = filesystem::path(config.getOutputPath()) / ("weights.bin");
    if(weightPath.exists()) {
        LOGI << "\tLoading weights from " << weightPath;

        const auto weights = readMatrix<InfoMaxWeightMatrixType::Scalar>(weightPath);
        return InfoMaxType(inputSize, weights);
    }
    else {
        return InfoMaxType(inputSize);
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
