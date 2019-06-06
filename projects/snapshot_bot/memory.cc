#define NO_HEADER_DEFINITIONS
#include "memory.h"

// Standard C++ includes
#include <fstream>

// BoB robotics includes
#include "common/logging.h"

// BoB robotics third party includes
#include "third_party/path.h"

// Snapshot bot includes
#include "config.h"

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

//------------------------------------------------------------------------
// MemoryBase
//------------------------------------------------------------------------
MemoryBase::MemoryBase()
:   m_BestHeading(0.0_deg), m_LowestDifference(std::numeric_limits<size_t>::max())
{
}
//------------------------------------------------------------------------
void MemoryBase::writeCSVHeader(std::ostream &os)
{
    os << "Best heading [degrees], Lowest difference";
}
//------------------------------------------------------------------------
void MemoryBase::writeCSVLine(std::ostream &os)
{
    os << getBestHeading().value() << ", " << getLowestDifference();
}

//------------------------------------------------------------------------
// PerfectMemory
//------------------------------------------------------------------------
PerfectMemory::PerfectMemory(const Config &config, const cv::Size &inputSize)
:   m_PM(inputSize), m_BestSnapshotIndex(std::numeric_limits<size_t>::max())
{
    // Load mask image
    if(!config.getMaskImageFilename().empty()) {
        getPM().setMaskImage(config.getMaskImageFilename());
    }
}
//------------------------------------------------------------------------
void PerfectMemory::test(const cv::Mat &snapshot)
{
    // Get heading directly from Perfect Memory
    degree_t bestHeading;
    float lowestDifference;
    std::tie(bestHeading, m_BestSnapshotIndex, lowestDifference, std::ignore) = getPM().getHeading(snapshot);

    // Set best heading and vector length
    setBestHeading(bestHeading);
    setLowestDifference(lowestDifference);
}
//------------------------------------------------------------------------
void PerfectMemory::train(const cv::Mat &snapshot)
{
    getPM().train(snapshot);
}
//------------------------------------------------------------------------
void PerfectMemory::writeCSVHeader(std::ostream &os)
{
    // Superclass
    MemoryBase::writeCSVHeader(os);

    os << ", Best snapshot index";
}
//------------------------------------------------------------------------
void PerfectMemory::writeCSVLine(std::ostream &os)
{
    // Superclass
    MemoryBase::writeCSVLine(os);

    os << ", " << getBestSnapshotIndex();
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
void PerfectMemoryConstrained::test(const cv::Mat &snapshot)
{
    // Get best heading from left side of scan
    degree_t leftBestHeading;
    float leftLowestDifference;
    size_t leftBestSnapshot;
    std::tie(leftBestHeading, leftBestSnapshot, leftLowestDifference, std::ignore) = getPM().getHeading(
        snapshot, 1, 0, m_NumScanColumns);

    // Get best heading from right side of scan
    degree_t rightBestHeading;
    float rightLowestDifference;
    size_t rightBestSnapshot;
    std::tie(rightBestHeading, rightBestSnapshot, rightLowestDifference, std::ignore) = getPM().getHeading(
        snapshot, 1, m_ImageWidth - m_NumScanColumns, m_ImageWidth);

    // Get lowest difference and best heading from across scans
    setLowestDifference(std::min(leftLowestDifference, rightLowestDifference) / 255.0f);
    setBestHeading((leftLowestDifference < rightLowestDifference) ? leftBestHeading : rightBestHeading);
    setBestSnapshotIndex((leftLowestDifference < rightLowestDifference) ? leftBestSnapshot : rightBestSnapshot);
}

//------------------------------------------------------------------------
// InfoMax
//------------------------------------------------------------------------
InfoMax::InfoMax(const Config &config, const cv::Size &inputSize)
:   m_InfoMax(createInfoMax(config, inputSize))
{
    BOB_ASSERT(config.getMaskImageFilename().empty());
    LOGI << "\tUsing " << Eigen::nbThreads() << " threads";
}
//------------------------------------------------------------------------
void InfoMax::test(const cv::Mat &snapshot)
{
    // Get heading directly from InfoMax
    degree_t bestHeading;
    float lowestDifference;
    std::tie(bestHeading, lowestDifference, std::ignore) = getInfoMax().getHeading(snapshot);

    // Set best heading and vector length
    setBestHeading(bestHeading);
    setLowestDifference(lowestDifference);
}
//-----------------------------------------------------------------------
void InfoMax::train(const cv::Mat &snapshot)
{
    getInfoMax().train(snapshot);
}
//-----------------------------------------------------------------------
void InfoMax::saveWeights(const std::string &filename) const
{
    // Write weights to disk
    std::ofstream netFile(filename, std::ios::binary);
    const int size[2] { (int) getInfoMax().getWeights().rows(), (int) getInfoMax().getWeights().cols() };
    netFile.write(reinterpret_cast<const char *>(size), sizeof(size));
    netFile.write(reinterpret_cast<const char *>(getInfoMax().getWeights().data()), getInfoMax().getWeights().size() * sizeof(float));
}
//-----------------------------------------------------------------------
InfoMax::InfoMaxType InfoMax::createInfoMax(const Config &config, const cv::Size &inputSize)
{
    const filesystem::path weightPath = filesystem::path(config.getOutputPath()) / ("weights" + config.getTestingSuffix() + ".bin");
    if(weightPath.exists()) {
        LOGI << "\tLoading weights from " << weightPath;

        std::ifstream is(weightPath.str(), std::ios::binary);
        if (!is.good()) {
            throw std::runtime_error("Could not open " + weightPath.str());
        }

        // The matrix size is encoded as 2 x int32_t
        int32_t size[2];
        is.read(reinterpret_cast<char *>(&size), sizeof(size));

        // Create data array and fill it
        InfoMaxWeightMatrixType weights(size[0], size[1]);
        is.read(reinterpret_cast<char*>(weights.data()), sizeof(float) * weights.size());

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
void InfoMaxConstrained::test(const cv::Mat &snapshot)
{
    // Get best heading from left side of scan
    degree_t leftBestHeading;
    float leftLowestDifference;
    std::tie(leftBestHeading, leftLowestDifference, std::ignore) = getInfoMax().getHeading(
        snapshot, 1, 0, m_NumScanColumns);

    // Get best heading from right side of scan
    degree_t rightBestHeading;
    float rightLowestDifference;
    std::tie(rightBestHeading, rightLowestDifference, std::ignore) = getInfoMax().getHeading(
        snapshot, 1, m_ImageWidth - m_NumScanColumns, m_ImageWidth);

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
            LOGI << "Creating PerfectMemoryConstrained";
            return std::make_unique<PerfectMemoryConstrained>(config, inputSize);
        }
        else {
            LOGI << "Creating PerfectMemory";
            return std::make_unique<PerfectMemory>(config, inputSize);
        }
    }
}
