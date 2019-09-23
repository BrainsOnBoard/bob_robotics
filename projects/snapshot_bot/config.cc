#include "config.h"

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

Config::Config()
  : m_UseHOG(false)
  , m_UseBinaryImage(false)
  , m_UseHorizonVector(false)
  , m_Train(true)
  , m_UseInfoMax(false)
  , m_SaveTestingDiagnostic(false)
  , m_StreamOutput(false)
  , m_MaxSnapshotRotateDegrees(180.0)
  , m_UnwrapRes(180, 50)
  , m_WatershedMarkerImageFilename("segmentation.png")
  , m_NumHOGOrientations(8)
  , m_NumHOGPixelsPerCell(10)
  , m_JoystickDeadzone(0.25f)
  , m_AutoTrain(false)
  , m_TrainInterval(100.0)
  , m_MotorCommandInterval(500.0)
  , m_ServerListenPort(BoBRobotics::Net::Connection::DefaultListenPort)
  , m_MoveSpeed(0.25)
  , m_TurnThresholds{ { units::angle::degree_t(5.0), 0.5f }, { units::angle::degree_t(10.0), 1.0f } }
  , m_UseViconTracking(false)
  , m_ViconTrackingPort(0)
  , m_ViconTrackingObjectName("norbot")
  , m_UseViconCaptureControl(false)
  , m_ViconCaptureControlPort(0)
{
}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
bool
Config::shouldUseHOG() const
{
    return m_UseHOG;
}
bool
Config::shouldUseBinaryImage() const
{
    return m_UseBinaryImage;
}
bool
Config::shouldUseHorizonVector() const
{
    return m_UseHorizonVector;
}
bool
Config::shouldTrain() const
{
    return m_Train;
}
bool
Config::shouldUseInfoMax() const
{
    return m_UseInfoMax;
}
bool
Config::shouldSaveTestingDiagnostic() const
{
    return m_SaveTestingDiagnostic;
}
bool
Config::shouldStreamOutput() const
{
    return m_StreamOutput;
}

units::angle::degree_t
Config::getMaxSnapshotRotateAngle() const
{
    return units::angle::degree_t(m_MaxSnapshotRotateDegrees);
}

const filesystem::path &
Config::getOutputPath() const
{
    return m_OutputPath;
}
const std::string &
Config::getTestingSuffix() const
{
    return m_TestingSuffix;
}

const cv::Size &
Config::getUnwrapRes() const
{
    return m_UnwrapRes;
}

const std::string &
Config::getMaskImageFilename() const
{
    return m_MaskImageFilename;
}
const std::string &
Config::getWatershedMarkerImageFilename() const
{
    return m_WatershedMarkerImageFilename;
}

int
Config::getNumHOGOrientations() const
{
    return m_NumHOGOrientations;
}
int
Config::getNumHOGPixelsPerCell() const
{
    return m_NumHOGPixelsPerCell;
}
int
Config::getHOGDescriptorSize() const
{
    return (getUnwrapRes().width * getUnwrapRes().height * getNumHOGOrientations()) / (getNumHOGPixelsPerCell() * getNumHOGPixelsPerCell());
}

float
Config::getJoystickDeadzone() const
{
    return m_JoystickDeadzone;
}

bool
Config::shouldAutoTrain() const
{
    return m_AutoTrain;
}
Config::Milliseconds
Config::getTrainInterval() const
{
    return m_TrainInterval;
}
Config::Milliseconds
Config::getMotorCommandInterval() const
{
    return m_MotorCommandInterval;
}

bool
Config::shouldUseViconTracking() const
{
    return m_UseViconTracking;
}
int
Config::getViconTrackingPort() const
{
    return m_ViconTrackingPort;
}
const std::string &
Config::getViconTrackingObjectName() const
{
    return m_ViconTrackingObjectName;
}

bool
Config::shouldUseViconCaptureControl() const
{
    return m_UseViconCaptureControl;
}
const std::string &
Config::getViconCaptureControlName() const
{
    return m_ViconCaptureControlName;
}
const std::string &
Config::getViconCaptureControlHost() const
{
    return m_ViconCaptureControlHost;
}
int
Config::getViconCaptureControlPort() const
{
    return m_ViconCaptureControlPort;
}
const std::string &
Config::getViconCaptureControlPath() const
{
    return m_ViconCaptureControlPath;
}

int
Config::getServerListenPort() const
{
    return m_ServerListenPort;
}

float
Config::getMoveSpeed() const
{
    return m_MoveSpeed;
}

float
Config::getTurnSpeed(units::angle::degree_t angleDifference) const
{
    const auto absoluteAngleDifference = units::math::fabs(angleDifference);

    // Loop through turn speed thresholds in descending order
    for (auto i = m_TurnThresholds.crbegin(); i != m_TurnThresholds.crend(); ++i) {
        // If the angle difference passes this threshold, return corresponding speed
        if (absoluteAngleDifference >= i->first) {
            return i->second;
        }
    }

    // No turning required!
    return 0.0f;
}

void
Config::write(cv::FileStorage &fs) const
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
    fs << "turnThresholds"
       << "[";
    for (const auto &t : m_TurnThresholds) {
        fs << "[" << t.first.value() << t.second << "]";
    }
    fs << "]";

    if (shouldUseViconTracking()) {
        fs << "viconTracking"
           << "{";
        fs << "port" << getViconTrackingPort();
        fs << "objectName" << getViconTrackingObjectName();
        fs << "}";
    }

    if (shouldUseViconCaptureControl()) {
        fs << "viconCaptureControl"
           << "{";
        fs << "name" << getViconCaptureControlName();
        fs << "host" << getViconCaptureControlHost();
        fs << "port" << getViconCaptureControlPort();
        fs << "path" << getViconCaptureControlPath();
        fs << "}";
    }
    fs << "}";
}

void
Config::read(const cv::FileNode &node)
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
    m_OutputPath = (std::string) outputPath;

    cv::String testingSuffix;
    cv::read(node["testingSuffix"], testingSuffix, m_TestingSuffix);
    m_TestingSuffix = (std::string) testingSuffix;

    cv::read(node["maxSnapshotRotateDegrees"], m_MaxSnapshotRotateDegrees, m_MaxSnapshotRotateDegrees);
    cv::read(node["unwrapRes"], m_UnwrapRes, m_UnwrapRes);

    cv::String maskImageFilename;
    cv::read(node["maskImageFilename"], maskImageFilename, m_MaskImageFilename);
    m_MaskImageFilename = (std::string) maskImageFilename;

    cv::String watershedMarkerImageFilename;
    cv::read(node["watershedMarkerImageFilename"], watershedMarkerImageFilename, m_WatershedMarkerImageFilename);
    m_WatershedMarkerImageFilename = (std::string) watershedMarkerImageFilename;

    cv::read(node["numHOGOrientations"], m_NumHOGOrientations, m_NumHOGOrientations);
    cv::read(node["numHOGPixelsPerCell"], m_NumHOGPixelsPerCell, m_NumHOGPixelsPerCell);
    cv::read(node["joystickDeadzone"], m_JoystickDeadzone, m_JoystickDeadzone);
    cv::read(node["serverListenPort"], m_ServerListenPort, m_ServerListenPort);
    cv::read(node["autoTrain"], m_AutoTrain, m_AutoTrain);
    cv::read(node["moveSpeed"], m_MoveSpeed, m_MoveSpeed);

    double trainInterval;
    cv::read(node["trainInterval"], trainInterval, m_TrainInterval.count());
    m_TrainInterval = (Milliseconds) trainInterval;

    double motorCommandInterval;
    cv::read(node["motorCommandInterval"], motorCommandInterval, m_MotorCommandInterval.count());
    m_MotorCommandInterval = (Milliseconds) motorCommandInterval;

    if (node["turnThresholds"].isSeq()) {
        m_TurnThresholds.clear();
        for (const auto &t : node["turnThresholds"]) {
            assert(t.isSeq() && t.size() == 2);
            m_TurnThresholds.emplace(units::angle::degree_t((double) t[0]), (float) t[1]);
        }
    }

    const auto &viconTracking = node["viconTracking"];
    if (viconTracking.isMap()) {
        m_UseViconTracking = true;
        viconTracking["port"] >> m_ViconTrackingPort;

        cv::String viconTrackingObjectName;
        viconTracking["objectName"] >> viconTrackingObjectName;
        m_ViconTrackingObjectName = (std::string) viconTrackingObjectName;
    }

    const auto &viconCaptureControl = node["viconCaptureControl"];
    if (viconCaptureControl.isMap()) {
        m_UseViconCaptureControl = true;

        cv::String viconCaptureControlName;
        cv::String viconCaptureControlHost;
        cv::String viconCaptureControlPath;
        viconCaptureControl["name"] >> viconCaptureControlName;
        viconCaptureControl["host"] >> viconCaptureControlHost;
        viconCaptureControl["port"] >> m_ViconCaptureControlPort;
        viconCaptureControl["path"] >> viconCaptureControlPath;

        m_ViconCaptureControlName = (std::string) viconCaptureControlName;
        m_ViconCaptureControlHost = (std::string) viconCaptureControlHost;
        m_ViconCaptureControlPath = (std::string) viconCaptureControlPath;
    }
}

void
write(cv::FileStorage &fs, const std::string &, const Config &config)
{
    config.write(fs);
}

void
read(const cv::FileNode &node, Config &x, const Config &defaultValue)
{
    if (node.empty()) {
        x = defaultValue;
    } else {
        x.read(node);
    }
}
