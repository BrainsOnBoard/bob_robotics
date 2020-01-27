#pragma once

// Standard C++ includes
#include <string>
#include <vector>

// Standard C includes
#include <cstdint>

// OpenCV
#include <opencv2/opencv.hpp>


//------------------------------------------------------------------------
// Config
//------------------------------------------------------------------------
class Config
{
public:
    Config() : m_LiveImagePort(2000), m_SnapshotPort(2001), m_Resolution(1920, 1080),
        m_SnapshotInterpolationMethod(cv::INTER_LINEAR), m_MaxSnapshotRotateDegrees(90.0), m_TrainingLiveRect(420, 150, 1080, 150),
        m_TrainingSnapshotRects{{100, 500, 720, 100}, {1100, 500, 720, 100}, {100, 650, 720, 100}, {1100, 650, 720, 100},
                                {100, 800, 720, 100}, {1100, 800, 720, 100}, {100, 950, 720, 100}, {1100, 950, 720, 100}},
        m_TestingBestSnapshotPort(2002), m_TestingLiveRect(420, 150, 1080, 150), m_TestingBestRect(420, 400, 1080, 150),
        m_TestingDifferenceRect(420, 650, 1080, 150), m_TestingRIDFRect(420, 900, 1080, 150), m_TestingRIDFAxisRect(445, 925, 1030, 100),
        m_TestingRIDFBackgroundColour(255, 255, 255), m_TestingRIDFLineColour(0, 0, 0), m_TestingRIDFLineThickness(4)
    {
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    uint16_t getLiveImagePort() const{ return m_LiveImagePort; }
    uint16_t getSnapshotPort() const{ return m_SnapshotPort; }
    const cv::Size &getResolution() const{ return m_Resolution; }
    cv::InterpolationFlags getSnapshotInterpolationMethod() const{ return (cv::InterpolationFlags)m_SnapshotInterpolationMethod; }
    units::angle::degree_t getMaxSnapshotRotateAngle() const{ return units::angle::degree_t(m_MaxSnapshotRotateDegrees); }

    const std::string &getTrainingBackgroundFilename() const{ return m_TrainingBackgroundFilename; }
    const cv::Rect &getTrainingLiveRect() const{ return m_TrainingLiveRect; }
    const std::vector<cv::Rect> &getTrainingSnapshotRects() const{ return m_TrainingSnapshotRects; }

    const std::string &getTestingBackgroundFilename() const{ return m_TestingBackgroundFilename; }
    uint16_t getTestingBestSnapshotPort() const{ return m_TestingBestSnapshotPort; }
    const cv::Rect &getTestingLiveRect() const{ return m_TestingLiveRect; }
    const cv::Rect &getTestingBestRect() const{ return m_TestingBestRect; }
    const cv::Rect &getTestingDifferenceRect() const{ return m_TestingDifferenceRect; }
    const cv::Rect &getTestingRIDFRect() const{ return m_TestingRIDFRect; }
    const cv::Rect &getTestingRIDFAxisRect() const{ return m_TestingRIDFAxisRect; }
    const cv::Vec3b &getTestingRIDFBackgroundColour() const{ return m_TestingRIDFBackgroundColour; }
    const cv::Vec3b &getTestingRIDFLineColour() const{ return m_TestingRIDFLineColour; }
    int getTestingRIDFLineThickness() const{ return m_TestingRIDFLineThickness; }

    void write(cv::FileStorage& fs) const
    {
        fs << "{";
        fs << "liveImagePort" << getLiveImagePort();
        fs << "snapshotPort" << getSnapshotPort();
        fs << "resolution" << getResolution();
        fs << "snapshotInterpolationMethod" << getSnapshotInterpolationMethod();
        fs << "maxSnapshotRotateDegrees" << getMaxSnapshotRotateAngle().value();

        fs << "training" << "{";
        {
            fs << "backgroundFilename" << getTrainingBackgroundFilename();
            fs << "liveRect" << getTrainingLiveRect();
            fs << "snapshotRects" << "[";
            for(const auto &r : getTrainingSnapshotRects()) {
                fs << r;
            }
            fs << "]";
        }
        fs << "}";

        fs << "testing" << "{";
        {
            fs << "backgroundFilename" << getTestingBackgroundFilename();
            fs << "bestSnapshotPort" << getTestingBestSnapshotPort();
            fs << "liveRect" << getTestingLiveRect();
            fs << "bestRect" << getTestingBestRect();
            fs << "differenceRect" << getTestingDifferenceRect();
            fs << "ridfRect" << getTestingRIDFRect();
            fs << "ridfAxisRect" << getTestingRIDFAxisRect();
            fs << "ridfBackgroundColour" << getTestingRIDFBackgroundColour();
            fs << "ridfLineColour" << getTestingRIDFLineColour();
            fs << "ridfLineThickness" << getTestingRIDFLineThickness();
        }
        fs << "}";
        fs << "}";
    }

    void read(const cv::FileNode &node)
    {
        // Read settings
        // **NOTE** we use cv::read rather than stream operators as we want to use current values as defaults
        cv::read(node["liveImagePort"], m_LiveImagePort, m_LiveImagePort);
        cv::read(node["snapshotPort"], m_SnapshotPort, m_SnapshotPort);
        cv::read(node["resolution"], m_Resolution, m_Resolution);
        cv::read(node["snapshotInterpolationMethod"], m_SnapshotInterpolationMethod, m_SnapshotInterpolationMethod);
        cv::read(node["maxSnapshotRotateDegrees"], m_MaxSnapshotRotateDegrees, m_MaxSnapshotRotateDegrees);

        const auto &training = node["training"];
        if(training.isMap()) {
            cv::read(training["backgroundFilename"], m_TrainingBackgroundFilename, m_TrainingBackgroundFilename);
            cv::read(training["liveRect"], m_TrainingLiveRect, m_TrainingLiveRect);
            if(training["snapshotRects"].isSeq()) {
                m_TrainingSnapshotRects.clear();
                for(const auto &t : training["snapshotRects"]) {
                    m_TrainingSnapshotRects.emplace_back();
                    t >> m_TrainingSnapshotRects.back();
                }
            }
        }

        const auto &testing = node["testing"];
        if(testing.isMap()) {
            cv::read(testing["backgroundFilename"], m_TestingBackgroundFilename, m_TestingBackgroundFilename);
            cv::read(testing["bestSnapshotPort"], m_TestingBestSnapshotPort, m_TestingBestSnapshotPort);
            cv::read(testing["liveRect"], m_TestingLiveRect, m_TestingLiveRect);
            cv::read(testing["bestRect"], m_TestingBestRect, m_TestingBestRect);
            cv::read(testing["differenceRect"], m_TestingDifferenceRect, m_TestingDifferenceRect);
            cv::read(testing["ridfRect"], m_TestingRIDFRect, m_TestingRIDFRect);
            cv::read(testing["ridfRect"], m_TestingRIDFAxisRect, m_TestingRIDFAxisRect);
            cv::read(testing["ridfBackgroundColour"], m_TestingRIDFBackgroundColour, m_TestingRIDFBackgroundColour);
            cv::read(testing["ridfLineColour"], m_TestingRIDFLineColour, m_TestingRIDFLineColour);
            cv::read(testing["ridfLineThickness"], m_TestingRIDFLineThickness, m_TestingRIDFLineThickness);
        }

    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // TCP/IP port to connect to in order to receive live images
    uint16_t m_LiveImagePort;

    // TCP/IP port to connect to in order to receive snapshot image
    uint16_t m_SnapshotPort;

    // Screen resolution of display
    cv::Size m_Resolution;

    // Which interpolation method should we use for upscaling snapshots
    int m_SnapshotInterpolationMethod;

    // Maximum (absolute) angle snapshots will be rotated by
    double m_MaxSnapshotRotateDegrees;

    // Filename for training background image
    std::string m_TrainingBackgroundFilename;

    // Rectangle within which to display live image when training
    cv::Rect m_TrainingLiveRect;

    // Rectangles within which to display trained snapshotss
    std::vector<cv::Rect> m_TrainingSnapshotRects;

    // Filename for testing background image
    std::string m_TestingBackgroundFilename;

    // TCP/IP port to connect to in order to receive best snapshot images
    uint16_t m_TestingBestSnapshotPort;

    // Rectangle within which to display live image when testing
    cv::Rect m_TestingLiveRect;

    // Rectangle within which to display best matching image when testing
    cv::Rect m_TestingBestRect;

    // Rectangle within which to display difference image when testing
    cv::Rect m_TestingDifferenceRect;

    // Rectangle within which to display RIDF
    cv::Rect m_TestingRIDFRect;

    // Rectangle within the RIDF rect to display axis
    cv::Rect m_TestingRIDFAxisRect;

    cv::Vec3b m_TestingRIDFBackgroundColour;

    cv::Vec3b m_TestingRIDFLineColour;

    int m_TestingRIDFLineThickness;
};

static inline void write(cv::FileStorage &fs, const std::string&, const Config &config)
{
    config.write(fs);
}

static inline void read(const cv::FileNode &node, Config &x, const Config& defaultValue = Config())
{
    if(node.empty()) {
        x = defaultValue;
    }
    else {
        x.read(node);
    }
}
