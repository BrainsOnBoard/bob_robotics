#include "config.h"

// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"

// Standard C++ includes
#include <limits>

using namespace BoBRobotics;
using Milliseconds = std::chrono::duration<double, std::milli>;

namespace {
// **YUCK** as well as std::strings, it also seems like OpenCV can't serialise size_t
int getIntegerSize(size_t value)
{
    if(value == std::numeric_limits<size_t>::max()) {
        return -1;
    }
    else {
        return static_cast<int>(value);
    }
}

void readIntegerSize(const cv::FileNode &node, size_t &value, size_t defaultValue)
{
    const int defaultIntValue = (defaultValue == std::numeric_limits<size_t>::max()) ? -1 : static_cast<int>(defaultValue);

    int intValue;
    cv::read(node, intValue, defaultIntValue);

    if(intValue == -1) {
        value = std::numeric_limits<size_t>::max();
    }
    else {
        value = static_cast<size_t>(intValue);
    }
}
}

Config::Config()
  : m_UseBinaryImage(false)
  , m_UseHorizonVector(false)
  , m_Train(true)
  , m_UseInfoMax(false)
  , m_SaveTestingDiagnostic(false)
  , m_StreamOutput(false)
  , m_ODK2(false)
  , m_Webcam(false)
  , m_DriveRobot(true)
  , m_OutputPath(BoBRobotics::Path::getProgramDirectory() / "training")
  , m_RecordVideo(false)
  , m_VideoFileExtension("avi")
  , m_VideoCodec("XVID")
  , m_MaxSnapshotRotate(180.0)
  , m_PMFwdLASize(std::numeric_limits<size_t>::max())
  , m_PMFwdConfig{ 0, 0, 0, 0 }
  , m_UnwrapRes(180, 50)
  , m_CroppedRect(0, 0, 180, 50)
  , m_WatershedMarkerImageFilename("segmentation.png")
  , m_JoystickDeadzone(0.25f)
  , m_JoystickGain(1.f)
  , m_AutoTrain(true)
  , m_TrainInterval(0.0)
  , m_MotorCommandInterval(500.0)
  , m_MotorTurnCommandInterval(500.0)
  , m_TestSkipFrames(1)
  , m_ServerListenPort(BoBRobotics::Net::Connection::DefaultListenPort)
  , m_SnapshotServerListenPort(BoBRobotics::Net::Connection::DefaultListenPort + 1)
  , m_BestSnapshotServerListenPort(BoBRobotics::Net::Connection::DefaultListenPort + 2)
  , m_MoveSpeed(0.25)
  , m_TurnThresholds{ { units::angle::degree_t(5.0), { 0.5f, Milliseconds(500.0) } }, { units::angle::degree_t(10.0), { 1.0f, Milliseconds(500.0) } } }
  , m_UseViconTracking(false)
  , m_ViconTrackingPort(0)
  , m_ViconTrackingObjectName("norbot")
  , m_UseViconCaptureControl(false)
  , m_ViconCaptureControlPort(0)
{}

std::pair<float, Milliseconds>
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
    return {};
}

void
Config::read(const cv::FileNode &node)
{
        // Read settings
        // **NOTE** we use cv::read rather than stream operators as we want to use current values as defaults
        cv::read(node["shouldUseBinaryImage"], m_UseBinaryImage, m_UseBinaryImage);
        cv::read(node["shouldUseHorizonVector"], m_UseHorizonVector, m_UseHorizonVector);
        cv::read(node["shouldTrain"], m_Train, m_Train);
        cv::read(node["shouldUseInfoMax"], m_UseInfoMax, m_UseInfoMax);
        cv::read(node["shouldSaveTestingDiagnostic"], m_SaveTestingDiagnostic, m_SaveTestingDiagnostic);
        cv::read(node["shouldStreamOutput"], m_StreamOutput, m_StreamOutput);
        cv::read(node["shouldUseODK2"], m_ODK2, m_ODK2);
        cv::read(node["shouldUseWebcam"], m_Webcam, m_Webcam);
        cv::read(node["shouldDriveRobot"], m_DriveRobot, m_DriveRobot);
        cv::read(node["shouldRecordVideo"], m_RecordVideo, m_RecordVideo);
        cv::read(node["videoCodec"], m_VideoCodec, m_VideoCodec);
        cv::read(node["videoFileExtension"], m_VideoFileExtension, m_VideoFileExtension);

        // Assert that configuration is valid
        BOB_ASSERT(!m_UseBinaryImage || !m_UseHorizonVector);

        // **YUCK** why does OpenCV (at least my version) not have a cv::read overload for std::string!?
        cv::String outputPath;
        cv::read(node["outputPath"], outputPath, m_OutputPath.str());
        if (outputPath.empty()) {
            m_OutputPath = filesystem::current_path() / "training";
        } else {
            m_OutputPath = (std::string)outputPath;
        }

        cv::read(node["maxSnapshotRotateDegrees"], m_MaxSnapshotRotate, m_MaxSnapshotRotate);

        readIntegerSize(node["pmFwdLASize"], m_PMFwdLASize, m_PMFwdLASize);
        readIntegerSize(node["pmFwdLAIncreaseSize"], m_PMFwdConfig.increaseSize, m_PMFwdConfig.increaseSize);
        readIntegerSize(node["pmFwdLADecreaseSize"], m_PMFwdConfig.decreaseSize, m_PMFwdConfig.decreaseSize);
        readIntegerSize(node["pmMinFwdLASize"], m_PMFwdConfig.minSize, m_PMFwdConfig.minSize);
        readIntegerSize(node["pmMaxFwdLASize"], m_PMFwdConfig.maxSize, m_PMFwdConfig.maxSize);

        cv::read(node["unwrapRes"], m_UnwrapRes, m_UnwrapRes);
        cv::read(node["croppedRect"], m_CroppedRect, m_CroppedRect);

        cv::String maskImageFilename;
        cv::read(node["maskImageFilename"], maskImageFilename, m_MaskImageFilename);
        m_MaskImageFilename = (std::string)maskImageFilename;

        cv::String watershedMarkerImageFilename;
        cv::read(node["watershedMarkerImageFilename"], watershedMarkerImageFilename, m_WatershedMarkerImageFilename);
        m_WatershedMarkerImageFilename = (std::string)watershedMarkerImageFilename;

        cv::read(node["joystickDeadzone"], m_JoystickDeadzone, m_JoystickDeadzone);
        cv::read(node["joystickGain"], m_JoystickGain, m_JoystickGain);
        cv::read(node["serverListenPort"], m_ServerListenPort, m_ServerListenPort);
        cv::read(node["snapshotServerListenPort"], m_SnapshotServerListenPort, m_SnapshotServerListenPort);
        cv::read(node["bestSnapshotServerListenPort"], m_BestSnapshotServerListenPort, m_BestSnapshotServerListenPort);
        cv::read(node["autoTrain"], m_AutoTrain, m_AutoTrain);
        cv::read(node["moveSpeed"], m_MoveSpeed, m_MoveSpeed);

        double trainInterval;
        cv::read(node["trainInterval"], trainInterval, m_TrainInterval.count());
        m_TrainInterval = (Milliseconds)trainInterval;

        int testFrameSkip = m_TestSkipFrames;
        cv::read(node["testFrameSkip"], testFrameSkip, testFrameSkip);
        BOB_ASSERT(testFrameSkip > 0);
        m_TestSkipFrames = (size_t) testFrameSkip;

        double motorCommandInterval;
        cv::read(node["motorCommandInterval"], motorCommandInterval, m_MotorCommandInterval.count());
        m_MotorCommandInterval = (Milliseconds)motorCommandInterval;
        cv::read(node["motorTurnCommandInterval"], motorCommandInterval, m_MotorTurnCommandInterval.count());
        m_MotorTurnCommandInterval = (Milliseconds)motorCommandInterval;

        if(node["turnThresholds"].isSeq()) {
            m_TurnThresholds.clear();
            for(const auto &t : node["turnThresholds"]) {
                assert(t.isSeq());

                if(t.size() == 2) {
                    m_TurnThresholds.emplace(std::piecewise_construct,
                                             std::forward_as_tuple(units::angle::degree_t((double)t[0])),
                                             std::forward_as_tuple((float)t[1], m_MotorTurnCommandInterval));
                }
                else if(t.size() == 3){
                    m_TurnThresholds.emplace(std::piecewise_construct,
                                             std::forward_as_tuple(units::angle::degree_t((double)t[0])),
                                             std::forward_as_tuple((float)t[1], (Milliseconds)t[2]));

                }
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

void
Config::write(cv::FileStorage &fs) const
{
    fs << "{";
    fs << "shouldUseBinaryImage" << shouldUseBinaryImage();
    fs << "shouldUseHorizonVector" << shouldUseHorizonVector();
    fs << "shouldTrain" << shouldTrain();
    fs << "shouldUseInfoMax" << shouldUseInfoMax();
    fs << "shouldSaveTestingDiagnostic" << shouldSaveTestingDiagnostic();
    fs << "shouldStreamOutput" << shouldStreamOutput();
    fs << "shouldUseODK2" << shouldUseODK2();
    fs << "shouldUseWebcam" << shouldUseWebcam();
    fs << "shouldDriveRobot" << shouldDriveRobot();
    fs << "outputPath" << getOutputPath().str();
    fs << "shouldRecordVideo" << shouldRecordVideo();
    fs << "videoCodec" << getVideoCodec();
    fs << "videoFileExtension" << getVideoFileExtension();
    fs << "maxSnapshotRotateDegrees" << getMaxSnapshotRotateAngle().value();
    fs << "pmFwdLASize" << getIntegerSize(getPMFwdLASize());
    fs << "pmFwdLAIncreaseSize" << getIntegerSize(getPMFwdConfig().increaseSize);
    fs << "pmFwdLADecreaseSize" << getIntegerSize(getPMFwdConfig().decreaseSize);
    fs << "pmMinFwdLASize" << getIntegerSize(getPMFwdConfig().minSize);
    fs << "pmMaxFwdLASize" << getIntegerSize(getPMFwdConfig().maxSize);
    fs << "unwrapRes" << getUnwrapRes();
    fs << "croppedRect" << getCroppedRect();
    fs << "maskImageFilename" << getMaskImageFilename();
    fs << "watershedMarkerImageFilename" << getWatershedMarkerImageFilename();
    fs << "joystickDeadzone" << getJoystickDeadzone();
    fs << "joystickGain" << getJoystickGain();
    fs << "autoTrain" << shouldAutoTrain();
    fs << "trainInterval" << getTrainInterval().count();
    fs << "testFrameSkip" << getIntegerSize(getSkipFrames());
    fs << "motorCommandInterval" << getMotorCommandInterval().count();
    fs << "motorTurnCommandInterval" << m_MotorTurnCommandInterval.count();
    fs << "serverListenPort" << getServerListenPort();
    fs << "snapshotServerListenPort" << getSnapshotServerListenPort();
    fs << "bestSnapshotServerListenPort" << getBestSnapshotServerListenPort();
    fs << "moveSpeed" << getMoveSpeed();
    fs << "turnThresholds"
       << "[";
    for (const auto &t : m_TurnThresholds) {
        fs << "[" << t.first.value() << t.second.first << t.second.second.count() << "]";
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
Config::parseArgs(int argc, char **argv)
{
    filesystem::path configPath{ "config.yaml" };
    bool configIsDatabase = false;

    if (argc > 1) {
        configPath = argv[1];
        configIsDatabase = configPath.is_directory();
        if (configIsDatabase) {
            configPath = configPath / "database_metadata.yaml";
            BOB_ASSERT(configPath.exists());
        }
    }

    // Read config values from file
    {
        cv::FileStorage configFile(configPath.str(), cv::FileStorage::READ);
        if(configFile.isOpened()) {
            if (configIsDatabase) {
                configFile["metadata"]["config"] >> *this;
            } else {
                configFile["config"] >> *this;
            }
        }
    }

    // Re-write config file
    if (!configIsDatabase) {
        cv::FileStorage configFile(configPath.str(), cv::FileStorage::WRITE);
        configFile << "config" << *this;
    }
}
