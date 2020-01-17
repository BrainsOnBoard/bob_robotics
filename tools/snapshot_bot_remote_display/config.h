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
    m_SnapshotInterpolationMethod(cv::INTER_LINEAR), m_TrainingLiveRect(420, 150, 1080, 150),
    m_TrainingSnapshotRects{{100, 500, 720, 100}, {1100, 500, 720, 100}, {100, 650, 720, 100}, {1100, 650, 720, 100},
                            {100, 800, 720, 100}, {1100, 800, 720, 100}, {100, 950, 720, 100}, {1100, 950, 720, 100}},
    m_TestingBestSnapshotPort(2002), m_TestingLiveRect(420, 150, 1080, 150), m_TestingBestRect(420, 400, 1080, 150),
    m_TestingDifferenceRect(420, 650, 1080, 150), m_TestingRIDFRect(420, 900, 1080, 150)
    {
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    uint16_t getLiveImagePort() const{ return m_LiveImagePort; }
    uint16_t getSnapshotPort() const{ return m_SnapshotPort; }
    const cv::Size &getResolution() const{ return m_Resolution; }
    cv::InterpolationFlags getSnapshotInterpolationMethod() const{ return m_SnapshotInterpolationMethod; }

    const std::string &getTrainingBackgroundFilename() const{ return m_TrainingBackgroundFilename; }
    const cv::Rect &getTrainingLiveRect() const{ return m_TrainingLiveRect; }
    const std::vector<cv::Rect> &getTrainingSnapshotRects() const{ return m_TrainingSnapshotRects; }

    const std::string &getTestingBackgroundFilename() const{ return m_TestingBackgroundFilename; }
    uint16_t getTestingBestSnapshotPort() const{ return m_TestingBestSnapshotPort; }
    const cv::Rect &getTestingLiveRect() const{ return m_TestingLiveRect; }
    const cv::Rect &getTestingBestRect() const{ return m_TestingBestRect; }
    const cv::Rect &getTestingDifferenceRect() const{ return m_TestingDifferenceRect; }
    const cv::Rect &getTestingRIDFRect() const{ return m_TestingRIDFRect; }

    void write(cv::FileStorage& fs) const
    {
        fs << "{";
        fs << "liveImagePort" << getLiveImagePort();
        fs << "snapshotPort" << getSnapshotPort();
        fs << "resolution" << getResolution();
        fs << "snapshotInterpolationMethod" << getSnapshotInterpolationMethod();

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

        const auto &training = node["training"];
        if(training.isMap()) {
            training["backgroundFilename"] >> m_TrainingBackgroundFilename;
            training["liveRect"] >> m_TrainingLiveRect;
            if(training["snapshotRects"].isSeq()) {
                m_TrainingSnapshotRects.clear();
                for(const auto &t : node["snapshotRects"]) {
                    m_TrainingSnapshotRects.emplace_back();
                    t >> m_TrainingSnapshotRects.back();
                }
            }
        }

        const auto &testing = node["testing"];
        if(testing.isMap()) {
            testing["backgroundFilename"] >> m_TestingBackgroundFilename;
            testing["bestSnapshotPort"] >> m_TestingBestSnapshotPort;
            testing["liveRect"] >> m_TestingLiveRect;
            testing["bestRect"] >> m_TestingBestRect;
            testing["differenceRect"] >> m_TestingDifferenceRect;
            testing["ridfRect"] >> m_TestingRIDFRect;
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
    cv::InterpolationFlags m_SnapshotInterpolationMethod;

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
