#pragma once

// BoB robotics includes
#include "net/connection.h"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <chrono>
#include <map>
#include <string>

//------------------------------------------------------------------------
// Config
//------------------------------------------------------------------------
class Config
{
    using Milliseconds = std::chrono::duration<double, std::milli>;

public:
    Config() : m_UseHOG(false), m_UseBinaryImage(false), m_UseHorizonVector(false), m_Train(true), m_UseInfoMax(false), m_SaveTestingDiagnostic(false), m_StreamOutput(false),
        m_MaxSnapshotRotateDegrees(180.0), m_UnwrapRes(180, 50), m_WatershedMarkerImageFilename("segmentation.png"), m_NumHOGOrientations(8), m_NumHOGPixelsPerCell(10),
        m_JoystickDeadzone(0.25f), m_AutoTrain(false), m_TrainInterval(100.0), m_MotorCommandInterval(500.0), m_ServerListenPort(BoBRobotics::Net::Connection::DefaultListenPort), m_MoveSpeed(0.25),
        m_TurnThresholds{{units::angle::degree_t(5.0), 0.5f}, {units::angle::degree_t(10.0), 1.0f}}, m_UseViconTracking(false), m_ViconTrackingPort(0), m_ViconTrackingObjectName("norbot"),
        m_UseViconCaptureControl(false), m_ViconCaptureControlPort(0)
    {
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool shouldUseHOG() const{ return m_UseHOG; }
    bool shouldUseBinaryImage() const{ return m_UseBinaryImage; }
    bool shouldUseHorizonVector() const{ return m_UseHorizonVector; }
    bool shouldTrain() const{ return m_Train; }
    bool shouldUseInfoMax() const{ return m_UseInfoMax; }
    bool shouldSaveTestingDiagnostic() const{ return m_SaveTestingDiagnostic; }
    bool shouldStreamOutput() const{ return m_StreamOutput; }

    units::angle::degree_t getMaxSnapshotRotateAngle() const{ return units::angle::degree_t(m_MaxSnapshotRotateDegrees); }

    const filesystem::path &getOutputPath() const{ return m_OutputPath; }
    const std::string &getTestingSuffix() const{ return m_TestingSuffix; }

    const cv::Size &getUnwrapRes() const{ return m_UnwrapRes; }


    const std::string &getMaskImageFilename() const{ return m_MaskImageFilename; }
    const std::string &getWatershedMarkerImageFilename() const{ return m_WatershedMarkerImageFilename; }

    int getNumHOGOrientations() const{ return m_NumHOGOrientations; }
    int getNumHOGPixelsPerCell() const{ return m_NumHOGPixelsPerCell; }
    int getHOGDescriptorSize() const{ return (getUnwrapRes().width * getUnwrapRes().height * getNumHOGOrientations()) / (getNumHOGPixelsPerCell() * getNumHOGPixelsPerCell()); }

    float getJoystickDeadzone() const{ return m_JoystickDeadzone; }

    bool shouldAutoTrain() const{ return m_AutoTrain; }
    Milliseconds getTrainInterval() const{ return m_TrainInterval; }
    Milliseconds getMotorCommandInterval() const{ return m_MotorCommandInterval; }

    bool shouldUseViconTracking() const{ return m_UseViconTracking; }
    int getViconTrackingPort() const{ return m_ViconTrackingPort; }
    const std::string &getViconTrackingObjectName() const{ return m_ViconTrackingObjectName; }

    bool shouldUseViconCaptureControl() const{ return m_UseViconCaptureControl; }
    const std::string &getViconCaptureControlName() const{ return m_ViconCaptureControlName; }
    const std::string &getViconCaptureControlHost() const{ return m_ViconCaptureControlHost; }
    int getViconCaptureControlPort() const { return m_ViconCaptureControlPort; }
    const std::string &getViconCaptureControlPath() const{ return m_ViconCaptureControlPath; }

    int getServerListenPort() const{ return m_ServerListenPort; }

    float getMoveSpeed() const{ return m_MoveSpeed; }

    float getTurnSpeed(units::angle::degree_t angleDifference) const
    {
        const auto absoluteAngleDifference = units::math::fabs(angleDifference);

        // Loop through turn speed thresholds in descending order
        for(auto i = m_TurnThresholds.crbegin(); i != m_TurnThresholds.crend(); ++i) {
            // If the angle difference passes this threshold, return corresponding speed
            if(absoluteAngleDifference >= i->first) {
                return i->second;
            }
        }

        // No turning required!
        return 0.0f;
    }


    void write(cv::FileStorage& fs) const
    {
        fs << "{";
        fs << "shouldUseHOG" << shouldUseHOG();
        fs << "shouldUseBinaryImage" << shouldUseBinaryImage();
        fs << "shouldUseHorizonVector" << shouldUseHorizonVector();
        fs << "shouldTrain" << shouldTrain();
        fs << "shouldUseInfoMax" << shouldUseInfoMax();
        fs << "shouldSaveTestingDiagnostic" << shouldSaveTestingDiagnostic();
        fs << "shouldStreamOutput" << shouldStreamOutput();
        fs << "outputPath" << getOutputPath().str();
        fs << "testingSuffix" << getTestingSuffix();
        fs << "maxSnapshotRotateDegrees" << getMaxSnapshotRotateAngle().value();
        fs << "unwrapRes" << getUnwrapRes();
        fs << "maskImageFilename" << getMaskImageFilename();
        fs << "watershedMarkerImageFilename" << getWatershedMarkerImageFilename();
        fs << "numHOGOrientations" << getNumHOGOrientations();
        fs << "numHOGPixelsPerCell" << getNumHOGPixelsPerCell();
        fs << "joystickDeadzone" << getJoystickDeadzone();
        fs << "autoTrain" << shouldAutoTrain();
        fs << "trainInterval" << getTrainInterval().count();
        fs << "motorCommandInterval" << getMotorCommandInterval().count();
        fs << "serverListenPort" << getServerListenPort();
        fs << "moveSpeed" << getMoveSpeed();
        fs << "turnThresholds" << "[";
        for(const auto &t : m_TurnThresholds) {
            fs << "[" << t.first.value() << t.second << "]";
        }
        fs << "]";

        if(shouldUseViconTracking()) {
            fs << "viconTracking" << "{";
            fs << "port" << getViconTrackingPort();
            fs << "objectName" << getViconTrackingObjectName();
            fs << "}";
        }

        if(shouldUseViconCaptureControl()) {
            fs << "viconCaptureControl" << "{";
            fs << "name" << getViconCaptureControlName();
            fs << "host" << getViconCaptureControlHost();
            fs << "port" << getViconCaptureControlPort();
            fs << "path" << getViconCaptureControlPath();
            fs << "}";
        }
        fs << "}";
    }

    void read(const cv::FileNode &node)
    {
        // Read settings
        // **NOTE** we use cv::read rather than stream operators as we want to use current values as defaults
        cv::read(node["shouldUseHOG"], m_UseHOG, m_UseHOG);
        cv::read(node["shouldUseBinaryImage"], m_UseBinaryImage, m_UseBinaryImage);
        cv::read(node["shouldUseHorizonVector"], m_UseHorizonVector, m_UseHorizonVector);
        cv::read(node["shouldTrain"], m_Train, m_Train);
        cv::read(node["shouldUseInfoMax"], m_UseInfoMax, m_UseInfoMax);
        cv::read(node["shouldSaveTestingDiagnostic"], m_SaveTestingDiagnostic, m_SaveTestingDiagnostic);
        cv::read(node["shouldStreamOutput"], m_StreamOutput, m_StreamOutput);

        // Assert that configuration is valid
        BOB_ASSERT(!m_UseBinaryImage || !m_UseHorizonVector);

        // **YUCK** why does OpenCV (at least my version) not have a cv::read overload for std::string!?
        cv::String outputPath;
        cv::read(node["outputPath"], outputPath, m_OutputPath.str());
        m_OutputPath = (std::string)outputPath;

        cv::String testingSuffix;
        cv::read(node["testingSuffix"], testingSuffix, m_TestingSuffix);
        m_TestingSuffix = (std::string)testingSuffix;

        cv::read(node["maxSnapshotRotateDegrees"], m_MaxSnapshotRotateDegrees, m_MaxSnapshotRotateDegrees);
        cv::read(node["unwrapRes"], m_UnwrapRes, m_UnwrapRes);

        cv::String maskImageFilename;
        cv::read(node["maskImageFilename"], maskImageFilename, m_MaskImageFilename);
        m_MaskImageFilename = (std::string)maskImageFilename;

        cv::String watershedMarkerImageFilename;
        cv::read(node["watershedMarkerImageFilename"], watershedMarkerImageFilename, m_WatershedMarkerImageFilename);
        m_WatershedMarkerImageFilename = (std::string)watershedMarkerImageFilename;

        cv::read(node["numHOGOrientations"], m_NumHOGOrientations, m_NumHOGOrientations);
        cv::read(node["numHOGPixelsPerCell"], m_NumHOGPixelsPerCell, m_NumHOGPixelsPerCell);
        cv::read(node["joystickDeadzone"], m_JoystickDeadzone, m_JoystickDeadzone);
        cv::read(node["serverListenPort"], m_ServerListenPort, m_ServerListenPort);
        cv::read(node["autoTrain"], m_AutoTrain, m_AutoTrain);
        cv::read(node["moveSpeed"], m_MoveSpeed, m_MoveSpeed);

        double trainInterval;
        cv::read(node["trainInterval"], trainInterval, m_TrainInterval.count());
        m_TrainInterval = (Milliseconds)trainInterval;

        double motorCommandInterval;
        cv::read(node["motorCommandInterval"], motorCommandInterval, m_MotorCommandInterval.count());
        m_MotorCommandInterval = (Milliseconds)motorCommandInterval;

        if(node["turnThresholds"].isSeq()) {
            m_TurnThresholds.clear();
            for(const auto &t : node["turnThresholds"]) {
                assert(t.isSeq() && t.size() == 2);
                m_TurnThresholds.emplace(units::angle::degree_t((double)t[0]), (float)t[1]);
            }
        }

        const auto &viconTracking = node["viconTracking"];
        if(viconTracking.isMap()) {
            m_UseViconTracking = true;
            viconTracking["port"] >> m_ViconTrackingPort;

            cv::String viconTrackingObjectName;
            viconTracking["objectName"] >> viconTrackingObjectName;
            m_ViconTrackingObjectName = (std::string)viconTrackingObjectName;
        }

        const auto &viconCaptureControl = node["viconCaptureControl"];
        if(viconCaptureControl.isMap()) {
            m_UseViconCaptureControl = true;

            cv::String viconCaptureControlName;
            cv::String viconCaptureControlHost;
            cv::String viconCaptureControlPath;
            viconCaptureControl["name"] >> viconCaptureControlName;
            viconCaptureControl["host"] >> viconCaptureControlHost;
            viconCaptureControl["port"] >> m_ViconCaptureControlPort;
            viconCaptureControl["path"] >> viconCaptureControlPath;

            m_ViconCaptureControlName = (std::string)viconCaptureControlName;
            m_ViconCaptureControlHost = (std::string)viconCaptureControlHost;
            m_ViconCaptureControlPath = (std::string)viconCaptureControlPath;
        }

    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Should we use HOG features or raw images?
    bool m_UseHOG;

    bool m_UseBinaryImage;

    bool m_UseHorizonVector;

    // Should we start in training mode or use existing data?
    bool m_Train;

    // Should we use InfoMax rather than Perfect Memory
    bool m_UseInfoMax;

    // Should we write out testing diagnostic information
    bool m_SaveTestingDiagnostic;

    // Should we transmit visual output
    bool m_StreamOutput;

    // Path to store snapshots etc
    filesystem::path m_OutputPath;

    // Suffix to add to end of testing images and csvs
    std::string m_TestingSuffix;

    // Maximum (absolute) angle snapshots will be rotated by
    double m_MaxSnapshotRotateDegrees;

    // What resolution to unwrap panoramas to?
    cv::Size m_UnwrapRes;

    // Filename of mask used to crop out unwanted bits of robot
    std::string m_MaskImageFilename;

    // Filename of image used to provide markers to watershed segmentation algorithm;
    std::string m_WatershedMarkerImageFilename;

    // HOG configuration
    int m_NumHOGOrientations;
    int m_NumHOGPixelsPerCell;

    // How large should the deadzone be on the analogue joystick?
    float m_JoystickDeadzone;

    // Should we train automatically every train interval
    bool m_AutoTrain;

    // How many milliseconds do we move for before re-calculating IDF?
    Milliseconds m_TrainInterval;
    Milliseconds m_MotorCommandInterval;

    // Listen port used for streaming etc
    int m_ServerListenPort;

    // How fast robot should move when heading to snapshot
    float m_MoveSpeed;

    // RDF angle difference thresholds that trigger different turning speeds
    std::map<units::angle::degree_t, float> m_TurnThresholds;

    // Vicon tracking settings
    bool m_UseViconTracking;
    int m_ViconTrackingPort;
    std::string m_ViconTrackingObjectName;

    // Vicon capture control settings
    bool m_UseViconCaptureControl;
    std::string m_ViconCaptureControlName;
    std::string m_ViconCaptureControlHost;
    int m_ViconCaptureControlPort;
    std::string m_ViconCaptureControlPath;
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
