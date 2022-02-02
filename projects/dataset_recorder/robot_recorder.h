#pragma once

// BoB robotics includes
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
#include <array>
#include <chrono>
#include <exception>
#include <memory>
#include <thread>

namespace BoBRobotics {
class RobotRecorder {
public:
    RobotRecorder(GPS::GPSReader &gps, bool useSystemClock)
      : m_GPS{ gps }
      , m_Camera{ getCamera() }
      , m_Database{ getCurrentDateTime(useSystemClock) }
      , m_Recorder{ m_Database.createVideoRouteRecorder(m_Camera->getOutputSize(),
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
        // Start timing
        m_Stopwatch.start();
    }

    ~RobotRecorder()
    {
        // Make sure that written data is actually written to disk before we exit
        ::sync();
    }

    //! Record from sensors. Returns time elapsed since start of recording.
    template<class T>
    auto step(T &robot, BN055 &imu)
    {
        using namespace units::angle;
        using namespace units::length;
        using namespace units::time;

        // Read image from camera (synchronously)
        m_Camera->readFrameSync(m_Frame);

        // Sync time to when camera frame was read
        const millisecond_t time = m_Stopwatch.elapsed();

        // get imu data
        constexpr degree_t nandeg{ NAN };
        std::array<degree_t, 3> attitude = { nandeg, nandeg, nandeg };
        try {
            attitude = imu.getEulerAngles();
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
        if (const auto optData = m_GPS.read()) {
            m_GPSData = optData.value();
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
            m_Recorder->record({ utm.toVector(), attitude }, m_Frame, botSpeed,
                               turnAngle.value(), utm.zone,
                               (int) m_GPSData.gpsQuality,
                               millimeter_t{ m_GPSData.horizontalDilution }.value(),
                               time.value());
        } else {
            /*
             * No new data was available. Indicate this by writing NaNs to the
             * CSV file. We can always estimate these missing values post hoc
             * with interpolation.
             */
            m_Recorder->record(Pose3<millimeter_t, degree_t>::nan(), m_Frame,
                               botSpeed, turnAngle.value(), "", -1, NAN,
                               time.value());
        }

        return time;
    }

    const auto &getGPSData() const
    {
        return m_GPSData;
    }

private:
    GPS::GPSReader &m_GPS;
    GPS::GPSData m_GPSData;
    std::unique_ptr<Video::Input> m_Camera;
    cv::Mat m_Frame;
    Navigation::ImageDatabase m_Database;
    std::unique_ptr<Navigation::ImageDatabase::RouteRecorder> m_Recorder;
    Stopwatch m_Stopwatch;

    static std::unique_ptr<Video::Input> getCamera()
    {
#ifdef DUMMY_CAMERA
        LOGW << "USING DUMMY CAMERA!!!!!!!!";
        auto cam = std::make_unique<Video::RandomInput<>>(cv::Size{ 360, 100 });
        cam->setFrameRate(units::frequency::hertz_t{ 30 });
        return cam;
#else
        return Video::getPanoramicCamera();
#endif
    }

    /*
     * Jetsons sadly don't record the time and date while powered off. We can
     * sometimes get the date from the GPS though (also sadly) we don't always
     * get given this information. So let's have a command-line option to force
     * using the system clock. The onus is then on the user to check that the
     * system clock's date is correct.
     *
     * We do however always get *time* information with GPS coordinates, so we
     * can at least rely on that being correct.
     */
    const std::tm &getCurrentDateTime(bool useSystemClock) const
    {
        using namespace std::literals;

        std::tm time = [&]() {
            auto time = m_GPS.getCurrentDateTime();

            // We already have current date from GPS
            if (time.tm_year) {
                return time;
            }

            // Get the date from the system clock (the time still comes from the GPS)
            if (useSystemClock) {
                LOGW << "Using system clock to get current date. This may not be correct!";
                const auto rawTime = std::time(nullptr);
                return *gmtime(&rawTime);
            }

            // See if we get a valid date message in the next 5s...
            Stopwatch sw;
            sw.start();
            while (sw.elapsed() < 5s) {
                time = m_GPS.getCurrentDateTime();
                if (time.tm_year) {
                    return time;
                }
            }

            // ...and if we didn't, give up
            throw std::runtime_error{ "Timed out waiting for date from GPS" };
        }();

        const auto rawTime = mktime(&time);
        return *localtime(&rawTime);
    }

}; // RobotRecorder
} // BoBRobotics
