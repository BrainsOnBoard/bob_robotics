#pragma once

// BoB robotics includes
#include "navigation/perfect_memory_window.h"
#include "net/connection.h"

// Third-party includes
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
    using WindowConfig = BoBRobotics::Navigation::PerfectMemoryWindow::DynamicBestMatchGradient::WindowConfig;

public:
    Config();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool shouldUseBinaryImage() const{ return m_UseBinaryImage; }
    bool shouldUseHorizonVector() const{ return m_UseHorizonVector; }
    bool shouldTrain() const{ return m_Train; }
    bool shouldUseInfoMax() const{ return m_UseInfoMax; }
    bool shouldSaveTestingDiagnostic() const{ return m_SaveTestingDiagnostic; }
    bool shouldStreamOutput() const{ return m_StreamOutput; }
    bool shouldUseODK2() const{ return m_ODK2; }
    bool shouldRecordVideo() const{ return m_RecordVideo; }

    const std::string &getVideoCodec() const{ return m_VideoCodec; }
    const std::string &getVideoFileExtension() const{ return m_VideoFileExtension; }

    units::angle::degree_t getMaxSnapshotRotateAngle() const{ return units::angle::degree_t(m_MaxSnapshotRotate); }

    size_t getPMFwdLASize() const{ return m_PMFwdLASize; }
    const WindowConfig &getPMFwdConfig() const{ return m_PMFwdConfig; }

    const filesystem::path &getOutputPath() const{ return m_OutputPath; }

    const cv::Size &getUnwrapRes() const{ return m_UnwrapRes; }
    const cv::Rect &getCroppedRect() const{ return m_CroppedRect; }

    const std::string &getMaskImageFilename() const{ return m_MaskImageFilename; }
    const std::string &getWatershedMarkerImageFilename() const{ return m_WatershedMarkerImageFilename; }

    float getJoystickDeadzone() const{ return m_JoystickDeadzone; }

    bool shouldAutoTrain() const{ return m_AutoTrain; }
    Milliseconds getTrainInterval() const{ return m_TrainInterval; }
    Milliseconds getMotorCommandInterval() const{ return m_MotorCommandInterval; }

    size_t getSkipFrames() const{ return m_TestSkipFrames; }

    bool shouldUseViconTracking() const{ return m_UseViconTracking; }
    int getViconTrackingPort() const{ return m_ViconTrackingPort; }
    const std::string &getViconTrackingObjectName() const{ return m_ViconTrackingObjectName; }

    bool shouldUseViconCaptureControl() const{ return m_UseViconCaptureControl; }
    const std::string &getViconCaptureControlName() const{ return m_ViconCaptureControlName; }
    const std::string &getViconCaptureControlHost() const{ return m_ViconCaptureControlHost; }
    int getViconCaptureControlPort() const { return m_ViconCaptureControlPort; }
    const std::string &getViconCaptureControlPath() const{ return m_ViconCaptureControlPath; }

    int getServerListenPort() const{ return m_ServerListenPort; }
    int getSnapshotServerListenPort() const{ return m_SnapshotServerListenPort; }
    int getBestSnapshotServerListenPort() const{ return m_BestSnapshotServerListenPort; }

    float getMoveSpeed() const{ return m_MoveSpeed; }

    std::pair<float, Milliseconds> getTurnSpeed(units::angle::degree_t angleDifference) const;

    void write(cv::FileStorage &fs) const;

    void read(const cv::FileNode &node);

private:

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
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

    // Should we use the ODK2 camera
    bool m_ODK2;

    // Path to store snapshots etc
    filesystem::path m_OutputPath;

    // Should we save image database as a video file
    bool m_RecordVideo;

    // File extension to use for video file
    std::string m_VideoFileExtension;

    // Video codec to use for video file
    std::string m_VideoCodec;

    // Maximum (absolute) angle snapshots will be rotated by
    double m_MaxSnapshotRotate;

    //! Initial size of perfect memory forward lookahead window
    size_t m_PMFwdLASize;

    //! Configuration for perfect memory forward lookahead window
    WindowConfig m_PMFwdConfig;

    // What resolution to unwrap panoramas to?
    cv::Size m_UnwrapRes;

    // Rectangle to crop unwrapped image into
    cv::Rect m_CroppedRect;

    // Filename of mask used to crop out unwanted bits of robot
    std::string m_MaskImageFilename;

    // Filename of image used to provide markers to watershed segmentation algorithm;
    std::string m_WatershedMarkerImageFilename;

    // How large should the deadzone be on the analogue joystick?
    float m_JoystickDeadzone;

    // Should we train automatically every train interval
    bool m_AutoTrain;

    // How many milliseconds do we move for before re-calculating IDF?
    Milliseconds m_TrainInterval;
    Milliseconds m_MotorCommandInterval;
    Milliseconds m_MotorTurnCommandInterval;

    // Number of frames to skip when training algo (e.g. 2 means every other
    // image is used)
    size_t m_TestSkipFrames;

    // Listen port used for streaming etc
    int m_ServerListenPort;
    int m_SnapshotServerListenPort;
    int m_BestSnapshotServerListenPort;

    // How fast robot should move when heading to snapshot
    float m_MoveSpeed;

    // RDF angle difference thresholds that trigger different turning speeds
    std::map<units::angle::degree_t, std::pair<float, Milliseconds>> m_TurnThresholds;

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
