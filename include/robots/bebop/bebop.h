#pragma once

// BoB robotics includes
#include "common/map_coordinate.h"
#include "common/pose.h"
#include "common/semaphore.h"
#include "hid/joystick.h"
#include "robots/uav.h"
#include "video/input.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// POSIX includes
#include <signal.h>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <atomic>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>

extern "C"
{
// ARSDK includes
#include <libARController/ARCONTROLLER_Error.h>
#include <libARController/ARCONTROLLER_Frame.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>

// ffmpeg includes
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
}

// These constants no longer seem to be defined on my machine - AD
#ifndef CODEC_CAP_TRUNCATED
#define CODEC_CAP_TRUNCATED AV_CODEC_CAP_TRUNCATED
#define CODEC_FLAG_TRUNCATED AV_CODEC_FLAG_TRUNCATED
#define CODEC_FLAG2_CHUNKS AV_CODEC_FLAG2_CHUNKS
#endif // CODEC_CAP_TRUNCATED

// https://github.com/libav/libav/commit/104e10fb426f903ba9157fdbfe30292d0e4c3d72
// https://github.com/libav/libav/blob/33d18982fa03feb061c8f744a4f0a9175c1f63ab/doc/APIchanges#L697
#if (LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54, 25, 0))
#define AV_CODEC_ID_H264 CODEC_ID_H264
#endif

// https://github.com/libav/libav/blob/33d18982fa03feb061c8f744a4f0a9175c1f63ab/doc/APIchanges#L653
#if (LIBAVCODEC_VERSION_INT < AV_VERSION_INT(51, 42, 0))
#define AV_PIX_FMT_YUV420P PIX_FMT_YUV420P
#endif

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
#define av_frame_alloc avcodec_alloc_frame
#define av_frame_free avcodec_free_frame
#endif

// these values are hardcoded for Bebop drones
#define BEBOP_IP_ADDRESS "192.168.42.1"
#define BEBOP_DISCOVERY_PORT 44444

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

template<class T>
T unan()
{
    return T{ std::numeric_limits<double>::quiet_NaN() };
}

//! Handlers which are called when the drone takes off or lands
using FlightEventHandler = std::function<void(bool takeoff)>;

//! Handler for when an action finishes
using ActionCompletedHandler = std::function<void(bool success)>;

/*
 * Simply throws a runtime_error with appropriate message if
 * err != ARCONTROLLER_OK.
 */
inline void
checkError(eARCONTROLLER_ERROR err)
{
    if (err != ARCONTROLLER_OK) {
        throw std::runtime_error(std::string("Controller error: ") +
                                 ARCONTROLLER_Error_ToString(err));
    }
}

//------------------------------------------------------------------------------
// BoBRobotics::Robots::Bebop
//------------------------------------------------------------------------------
/*!
 * \brief An interface to Parrot Bebop 2 drones
 *
 * This class handles connection/disconnection and sending steering commands.
 * It also provides an interface to access the drone's video stream.
 */
//------------------------------------------------------------------------------
class Bebop
  : public UAV
{
    using ControllerPtr = std::unique_ptr<ARCONTROLLER_Device_t, std::function<void(ARCONTROLLER_Device_t *)>>;

    using radian_t = units::angle::radian_t;
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    //! Interface to the drone's built-in camera
    class VideoStream : public Video::Input
    {
    public:
        VideoStream(Bebop &bebop);
        virtual ~VideoStream() override;
        virtual bool readFrame(cv::Mat &) override;
        virtual cv::Size getOutputSize() const override;
        virtual std::string getCameraName() const override;

    private:
        cv::Mat m_Frame;
        std::mutex m_FrameMutex;
        bool m_NewFrame = false;
        bool m_CodecInitialised = false;
        bool m_FirstFrameReceived = false;
        AVCodecContext *m_CodecContextPtr = nullptr;
        AVCodec *m_CodecPtr = nullptr;
        AVFrame *m_FramePtr = nullptr;
        AVFrame *m_FrameBGRPtr = nullptr;
        AVPacket m_Packet;
        SwsContext *m_ImgConvertContextPtr = nullptr;
        uint8_t *m_FrameBGRRawPtr = nullptr;
        bool m_UpdateCodecParams = false;
        std::vector<uint8_t> m_CodecData;

        bool initCodec();
        bool reallocateBuffers();
        void cleanupBuffers();
        void reset();
        void convertFrameToBGR();
        bool setH264Params(uint8_t *sps_buffer_ptr,
                           uint32_t sps_buffer_size,
                           uint8_t *pps_buffer_ptr,
                           uint32_t pps_buffer_size);
        bool decode(const ARCONTROLLER_Frame_t *framePtr);
        int getFrameWidth() const
        {
            return m_CodecInitialised ? m_CodecContextPtr->width : 0;
        }
        int getFrameHeight() const
        {
            return m_CodecInitialised ? m_CodecContextPtr->height : 0;
        }

        static eARCONTROLLER_ERROR configCallback(ARCONTROLLER_Stream_Codec_t codec, void *data);
        static eARCONTROLLER_ERROR frameCallback(ARCONTROLLER_Frame_t *frame, void *data);
    }; // VideoStream

    class AnimationError
      : public std::runtime_error {
    public:
        AnimationError()
          : std::runtime_error("The animation failed to start")
        {}
    };

    /**!
     * \brief GPS info returned by drone
     *
     * The error values are standard deviations.
     */
    struct GPSData
    {
        MapCoordinate::GPSCoordinate coordinate{ unan<degree_t>(), unan<degree_t>(), unan<meter_t>() };
        meter_t latError{ unan<meter_t>() }, lonError{ unan<meter_t>() }, heightError{ unan<meter_t>() };
        int numberOfSatellites = 0;
    };

    //! The state of an animation maneuvre
    enum class AnimationState {
        Idle = ARCOMMANDS_ANIMATION_STATE_IDLE,
        Running = ARCOMMANDS_ANIMATION_STATE_RUNNING,
        Cancelling = ARCOMMANDS_ANIMATION_STATE_CANCELING
    };

    //! The drone's current state
    enum class State
    {
        Stopped = ARCONTROLLER_DEVICE_STATE_STOPPED,
        Starting = ARCONTROLLER_DEVICE_STATE_STARTING,
        Running = ARCONTROLLER_DEVICE_STATE_RUNNING,
        Paused = ARCONTROLLER_DEVICE_STATE_PAUSED,
        Stopping = ARCONTROLLER_DEVICE_STATE_STOPPING
    };

    //! State of call to relativeMove()
    enum class RelativeMoveState
    {
        Initial = -2,
        Moving = -1,
        Success = ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_OK,
        ErrorUnknown = ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_UNKNOWN,
        ErrorBusy = ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_BUSY,
        ErrorNotAvailable = ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_NOTAVAILABLE,
        ErrorInterrupted = ARCOMMANDS_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR_INTERRUPTED
    };

    enum class MoveToState {
        Running = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_RUNNING,
        Done = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_DONE,
        Cancelled = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_CANCELED,
        Error = ARCOMMANDS_ARDRONE3_PILOTINGSTATE_MOVETOCHANGED_STATUS_ERROR
    };

    Bebop(degrees_per_second_t maxYawSpeed = DefaultMaximumYawSpeed,
          meters_per_second_t maxVerticalSpeed = DefaultMaximumVerticalSpeed,
          degree_t maxTilt = DefaultMaximumTilt);
    virtual ~Bebop() override;

    /**!
     * \brief Get current GPS position and info
     *
     * Returns true if new data is available, false otherwise.
     */
    bool getGPSData(GPSData &gps);

    //! Get current state of moveTo() operation
    MoveToState getMoveToState() const;

    // speed limits
    degree_t getMaximumTilt() const;
    std::pair<degree_t, degree_t> &getTiltLimits();
    meters_per_second_t getMaximumVerticalSpeed() const;
    std::pair<meters_per_second_t, meters_per_second_t> &getVerticalSpeedLimits();
    degrees_per_second_t getMaximumYawSpeed() const;
    std::pair<degrees_per_second_t, degrees_per_second_t> &getYawSpeedLimits();

    // motor control
    virtual void takeOff() override;
    virtual void land() override;
    virtual void setPitch(float pitch) override;
    virtual void setRoll(float right) override;
    virtual void setVerticalSpeed(float up) override;
    virtual void setYawSpeed(float right) override;
    void relativeMove(meter_t x, meter_t y, meter_t z, radian_t yaw = 0_rad);
    void moveTo(const MapCoordinate::GPSCoordinate &coords,
                ActionCompletedHandler callback);
    void moveTo(const MapCoordinate::GPSCoordinate &coords, degree_t heading = unan<degree_t>(),
                ActionCompletedHandler callback = nullptr);
    void cancelMoveTo() const;

    RelativeMoveState getRelativeMoveState() const;
    std::pair<Vector3<meter_t>, radian_t> getRelativeMovePoseDifference() const;
    void resetRelativeMoveState();
    void startRecordingVideo();
    void stopRecordingVideo();
    std::pair<radians_per_second_t, radian_t> startHorizontalPanoramaAnimation(radians_per_second_t rotationSpeed,
                                                                               ActionCompletedHandler handler);
    std::pair<radians_per_second_t, radian_t> startHorizontalPanoramaAnimation(radians_per_second_t rotationSpeed = radians_per_second_t{ std::numeric_limits<double>::quiet_NaN() },
                                                                               radian_t rotationAngle = degree_t{ 360 },
                                                                               ActionCompletedHandler handler = nullptr);
    AnimationState getAnimationState() const;
    void cancelCurrentAnimation();
    bool isVideoRecording() const;

    // calibration
    void doFlatTrimCalibration();

    // misc
    float getBatteryLevel();
    State getState();
    VideoStream &getVideoStream();
    void takePhoto();
    void setFlightEventHandler(FlightEventHandler);

    //! Default maximum tilt for pitch and roll.
    static constexpr auto DefaultMaximumTilt = 8_deg;

    //! Default maximum yaw speed.
    static constexpr auto DefaultMaximumYawSpeed = 100_deg_per_s;

    //! Default maximum vertical speed.
    static constexpr auto DefaultMaximumVerticalSpeed = 1_mps;

private:
    //! A wrapper object for ARSDK dictionaries, to make it cleaner + safer
    class ARDict
    {
    public:
        ARDict(const ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);

        template<class T>
        bool get(T &out, const char *key) const
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
            HASH_FIND_STR(m_Element->arguments, key, arg);
            if (!arg) {
                return false;
            } else {
                out = getInternal<T>(arg->value);
                return true;
            }
        }

        template<class T>
        T getDefault(T defVal, const char *key) const
        {
            T out;
            return get(out, key) ? out : defVal;
        }

    private:
        const ARCONTROLLER_DICTIONARY_ELEMENT_t * const m_Element;

        template<class T>
        static T getInternal(const ARCONTROLLER_DICTIONARY_VALUE_t &value);
    };

    template<class UnitType>
    class LimitValues
    {
    public:
        UnitType m_UserMaximum;

        void onChanged(const Bebop::ARDict &dict,
                       const char *currentKey,
                       const char *minKey,
                       const char *maxKey)
        {
            float val;

            // get current value
            if (dict.get(val, currentKey)) {
                m_Current = units::make_unit<UnitType>(val);
            }

            // get min value
            if (dict.get(val, minKey)) {
                m_Limits.first = units::make_unit<UnitType>(val);
            }

            // get max value
            if (dict.get(val, maxKey)) {
                m_Limits.second = units::make_unit<UnitType>(val);
            }

            /*
             * Notify waiting threads. The if statement is necessary so that
             * we don't report the current maximum before we've had a chance
             * to set it to the user-preferred value.
             */
            if (m_Current == m_UserMaximum) {
                m_Semaphore.notify();
            }
        }

        UnitType getCurrent() const
        {
            m_Semaphore.waitOnce();
            return m_Current;
        }

        std::pair<UnitType, UnitType> &getLimits()
        {
            m_Semaphore.waitOnce();
            return m_Limits;
        }

    private:
        UnitType m_Current;
        std::pair<UnitType, UnitType> m_Limits;
        mutable Semaphore m_Semaphore;
    };


    GPSData m_GPSData;
    Semaphore m_StateSemaphore, m_FlatTrimSemaphore, m_BatteryLevelSemaphore,
            m_VideoRecordingSemaphore, m_HorizontalAnimationInfoSemaphore,
            m_MoveToSemaphore;
    LimitValues<degree_t> m_TiltLimits;
    LimitValues<meters_per_second_t> m_VerticalSpeedLimits;
    LimitValues<degrees_per_second_t> m_YawSpeedLimits;
    std::atomic<RelativeMoveState> m_RelativeMoveState{ RelativeMoveState::Initial };
    std::atomic<MoveToState> m_MoveToState{ MoveToState::Done };
    eARCOMMANDS_ARDRONE3_MEDIARECORDSTATE_VIDEOSTATECHANGEDV2_ERROR m_VideoRecordingError;
    Vector3<meter_t> m_RelativeMovePositionDistance{ 0_m, 0_m, 0_m };
    radian_t m_RelativeMoveAngleDistance{ 0_rad };
    radian_t m_HorizontalAnimationAngle;
    radians_per_second_t m_HorizontalAnimationSpeed;
    ControllerPtr m_Device;
    std::unique_ptr<VideoStream> m_VideoStream;
    FlightEventHandler m_FlightEventHandler = nullptr;
    ActionCompletedHandler m_AnimationCompletedCallback = nullptr;
    ActionCompletedHandler m_MoveToCompletedCallback = nullptr;
    std::mutex m_AnimationMutex, m_GPSDataMutex;
    AnimationState m_AnimationState = AnimationState::Idle;
    bool m_IsVideoRecording = false, m_GPSDataUpdated = false;
    std::atomic<unsigned char> m_BatteryLevel;

    void connect();
    void disconnect();
    void startStreaming();
    void stopStreaming();
    void addEventHandlers();
    void onBatteryChanged(const ARDict &dict);
    void onVideoRecordingStateChanged(const ARDict &dict);
    void onHorizontalPanoramaStateChanged(const ARDict &dict);
    void onGPSLocationChanged(const ARDict &dict);
    void onMoveToStateChanged(const ARDict &dict);
    void createControllerDevice();
    State getStateUpdate();

    // speed limits
    void setMaximumTilt(degree_t newValue);
    void setMaximumVerticalSpeed(meters_per_second_t newValue);
    void setMaximumYawSpeed(degrees_per_second_t newValue);

    static void commandReceived(eARCONTROLLER_DICTIONARY_KEY key,
                                ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                                void *data);
    void relativeMoveEnded(const ARDict &dict);
    static void alertStateChanged(const ARDict &dict);
    static void productVersionReceived(const ARDict &dict);
    static void magnetometerCalibrationStateReceived(const ARDict &dict);
    static int printCallback(eARSAL_PRINT_LEVEL level,
                             const char *tag,
                             const char *format,
                             va_list va);
    static void stateChanged(eARCONTROLLER_DEVICE_STATE newstate,
                             eARCONTROLLER_ERROR err,
                             void *data);
}; // Bebop
} // Robots
} // BoBRobotics
