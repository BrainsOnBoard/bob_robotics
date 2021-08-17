#pragma once

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/stopwatch.h"
#include "gps/gps_reader.h"
#include "navigation/image_database.h"

#ifdef DUMMY_CAMERA
#include "video/randominput.h"
#else
#include "video/panoramic.h"
#endif

// Third-party includes
#include "third_party/units.h"
#include "plog/Log.h"

// POSIX includes
#include <unistd.h>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <exception>
#include <memory>

namespace BoBRobotics {
class RobotRecorder {
public:
    RobotRecorder()
      : m_Camera{ getCamera() }
      , m_Database{ getCurrentTime() }
      , m_Recorder{ m_Database.getRouteVideoRecorder(m_Camera->getOutputSize(),
                                                     m_Camera->getFrameRate(),
                                                     "mp4",
                                                     "mp4v",
                                                     { "Pitch [degrees]",
                                                       "Roll [degrees]",
                                                       "Speed",
                                                       "Steering angle [degrees]",
                                                       "UTM zone",
                                                       "GPS quality",
                                                       "Horizontal dilution [mm]",
                                                       "Timestamp [ms]" }) }
    {
        // We're polling the GPS
        m_GPS.setBlocking(false);

        // Turn Ctrl+C etc. into exceptions
        m_Catcher.trapSignals();
    }

    ~RobotRecorder()
    {
        // Make sure that written data is actually written to disk before we exit
        ::sync();
    }

    //! Record from sensors. Returns time elapsed since start of recording.
    template<class T>
    auto step(T &robot)
    {
        using namespace units::angle;
        using namespace units::length;
        using namespace units::time;

        // Check for signals
        m_Catcher.check();

        // Read image from camera (synchronously)
        m_Camera->readFrameSync(m_Frame);

        // Sync time to when camera frame was read
        const millisecond_t time = m_Stopwatch.elapsed();

        // get imu data
        degree_t yaw{ NAN }, pitch{ NAN }, roll{ NAN };
        try {
            const auto angles = m_IMU.getEulerAngles();
            yaw = angles[0];
            pitch = angles[1];
            roll = angles[2];
        } catch (std::exception &e) {
            LOGE << "Could not read from IMU: " << e.what();
        }

        // read speed and steering angle from robot
        float botSpeed = NAN;
        degree_t turnAngle{ NAN };
        try {
            std::tie(botSpeed, turnAngle) = robot.readRemoteControl();
        } catch (std::exception &e) {
            // if we can't read speed or angle, we just write nan values
            LOGE << "Could not read speed and steering angle from robot : " << e.what();
        }

        // Poll for GPS data
        if (m_GPS.read(m_GPSData)) {
            const auto &coord = m_GPSData.coordinate;
            int gpsQual = (int) m_GPSData.gpsQuality; // gps quality

            // output results
            LOGD << std::setprecision(10)
                << "GPS quality: " << gpsQual
                << " latitude: " << coord.lat.value()
                << " longitude: " << coord.lon.value()
                << " num sats: " << m_GPSData.numberOfSatellites
                << " time: " << time.value() << std::endl;

            // converting to UTM
            const auto utm = MapCoordinate::latLonToUTM(coord);
            m_Recorder.record(utm.toVector(), yaw, m_Frame, pitch.value(),
                            roll.value(), botSpeed, turnAngle.value(), utm.zone,
                            (int) m_GPSData.gpsQuality,
                            millimeter_t{ m_GPSData.horizontalDilution }.value(),
                            time.value());
        } else {
            /*
             * No new data was available. Indicate this by writing NaNs to the
             * CSV file. We can always estimate these missing values post hoc
             * with interpolation.
             */
            m_Recorder.record(Vector3<millimeter_t>::nan(), yaw,
                m_Frame, pitch.value(), roll.value(), botSpeed,
                turnAngle.value(), "", -1, NAN, time.value());
        }

        return time;
    }

private:
    GPS::GPSReader m_GPS;
    GPS::GPSData m_GPSData;
    BN055 m_IMU;
    std::unique_ptr<Video::Input> m_Camera;
    cv::Mat m_Frame;
    Navigation::ImageDatabase m_Database;
    Navigation::ImageDatabase::RouteRecorder<Navigation::ImageDatabase::VideoFileWriter> m_Recorder;
    BackgroundExceptionCatcher m_Catcher;
    Stopwatch m_Stopwatch;

    static std::unique_ptr<Video::Input> getCamera()
    {
#ifdef DUMMY_CAMERA
        LOGW << "USING DUMMY CAMERA!!!!!!!!";
        return std::make_unique<Video::RandomInput<>>(cv::Size{ 360, 100 });
#else
        return Video::getPanoramicCamera();
#endif
    }

    const std::tm &getCurrentTime() const
    {
        /*
         * Use GPS to get time and date, because Jetsons don't remember the system
         * time across boots.
         *
         * As the GPS gives the current time in UTC (and we might be in BST in the
         * UK), we convert to localtime first.
         */
        std::tm currentTime = m_GPS.getCurrentDateTime();
        const auto time = mktime(&currentTime); // get raw time
        return *localtime(&time);
    }

}; // RobotRecorder
} // BoBRobotics
