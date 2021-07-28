#pragma once

// BoB robotics includes
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_window.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// Standard C++ includes
#include <fstream>

// Forward declarations
class Config;

namespace BoBRobotics
{
namespace ImgProc
{
class Mask;
}
}

//------------------------------------------------------------------------
// MemoryBase
//------------------------------------------------------------------------
class MemoryBase
{
public:
    MemoryBase();
    virtual ~MemoryBase() = default;

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual void test(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) = 0;
    virtual void train(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) = 0;

    virtual void writeCSVHeader(std::ostream &os);
    virtual void writeCSVLine(std::ostream &os);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    units::angle::degree_t getBestHeading() const{ return m_BestHeading; }
    float getLowestDifference() const{ return m_LowestDifference; }

protected:
    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    void setBestHeading(units::angle::degree_t bestHeading){ m_BestHeading = bestHeading; }
    void setLowestDifference(float lowestDifference){ m_LowestDifference = lowestDifference; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    units::angle::degree_t m_BestHeading;
    float m_LowestDifference;
};

//------------------------------------------------------------------------
// PerfectMemory
//------------------------------------------------------------------------
class PerfectMemory : public MemoryBase
{
public:
    PerfectMemory(const Config &config, const cv::Size &inputSize);

    //------------------------------------------------------------------------
    // MemoryBase virtuals
    //------------------------------------------------------------------------
    virtual void test(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;
    virtual void train(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;

    virtual void writeCSVHeader(std::ostream &os) override;
    virtual void writeCSVLine(std::ostream &os) override;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getBestSnapshotIndex() const{ return m_BestSnapshotIndex; }
    const cv::Mat &getBestSnapshot() const{ return getPM().getSnapshot(getBestSnapshotIndex()); }

protected:
    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    BoBRobotics::Navigation::PerfectMemoryRotater<> &getPM(){ return m_PM; }
    const BoBRobotics::Navigation::PerfectMemoryRotater<> &getPM() const{ return m_PM; }

    void setBestSnapshotIndex(size_t bestSnapshotIndex){ m_BestSnapshotIndex = bestSnapshotIndex; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    BoBRobotics::Navigation::PerfectMemoryRotater<> m_PM;
    size_t m_BestSnapshotIndex;
};

//------------------------------------------------------------------------
// PerfectMemoryConstrained
//------------------------------------------------------------------------
class PerfectMemoryConstrained : public PerfectMemory
{
public:
    PerfectMemoryConstrained(const Config &config, const cv::Size &inputSize);

    //------------------------------------------------------------------------
    // MemoryBase virtuals
    //------------------------------------------------------------------------
    virtual void test(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const int m_ImageWidth;
    const size_t m_NumScanColumns;
};

//------------------------------------------------------------------------
// PerfectMemoryConstrainedDynamicWindow
//------------------------------------------------------------------------
class PerfectMemoryConstrainedDynamicWindow : public PerfectMemory
{
public:
    PerfectMemoryConstrainedDynamicWindow(const Config &config, const cv::Size &inputSize);

    //------------------------------------------------------------------------
    // MemoryBase virtuals
    //------------------------------------------------------------------------
    virtual void test(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;

    virtual void writeCSVHeader(std::ostream &os) override;
    virtual void writeCSVLine(std::ostream &os) override;
private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const int m_ImageWidth;
    const size_t m_NumScanColumns;
    BoBRobotics::Navigation::PerfectMemoryWindow::DynamicBestMatchGradient m_Window;
};

//------------------------------------------------------------------------
// InfoMax
//------------------------------------------------------------------------
class InfoMax : public MemoryBase
{
    using InfoMaxType = BoBRobotics::Navigation::InfoMaxRotater<float>;
    using InfoMaxWeightMatrixType = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

public:
    InfoMax(const Config &config, const cv::Size &inputSize);

    virtual void test(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;
    virtual void train(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;

    void saveWeights(const filesystem::path &filename) const;

protected:
    //------------------------------------------------------------------------
    // Protected API
    //------------------------------------------------------------------------
    InfoMaxType &getInfoMax(){ return m_InfoMax; }
    const InfoMaxType &getInfoMax() const { return m_InfoMax; }

private:
    static InfoMaxType createInfoMax(const Config &config, const cv::Size &inputSize);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    InfoMaxType m_InfoMax;
};

//------------------------------------------------------------------------
// InfoMaxConstrained
//------------------------------------------------------------------------
class InfoMaxConstrained : public InfoMax
{
public:
    InfoMaxConstrained(const Config &config, const cv::Size &inputSize);

    virtual void test(const cv::Mat &snapshot, const BoBRobotics::ImgProc::Mask &mask) override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const int m_ImageWidth;
    const size_t m_NumScanColumns;
};

std::unique_ptr<MemoryBase> createMemory(const Config &config, const cv::Size &inputSize);
