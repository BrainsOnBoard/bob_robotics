#pragma once

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "common/pose.h"
#include "common/semaphore.h"
#include "hid/joystick.h"
#include "video/input.h"
#include "robots/uav.h"

// Third-party includes
#include "third_party/units.h"

// POSIX includes
#include <signal.h>

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

//! Handlers which are called when the drone takes off or lands
using FlightEventHandler = std::function<void(bool takeoff)>;

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
        inline int getFrameWidth() const
        {
            return m_CodecInitialised ? m_CodecContextPtr->width : 0;
        }
        inline int getFrameHeight() const
        {
            return m_CodecInitialised ? m_CodecContextPtr->height : 0;
        }

        static eARCONTROLLER_ERROR configCallback(ARCONTROLLER_Stream_Codec_t codec, void *data);
        static eARCONTROLLER_ERROR frameCallback(ARCONTROLLER_Frame_t *frame, void *data);
    }; // VideoStream

    //! The drone's current state
    enum class State
    {
        Stopped = ARCONTROLLER_DEVICE_STATE_STOPPED,
        Starting = ARCONTROLLER_DEVICE_STATE_STARTING,
        Running = ARCONTROLLER_DEVICE_STATE_RUNNING,
        Paused = ARCONTROLLER_DEVICE_STATE_PAUSED,
        Stopping = ARCONTROLLER_DEVICE_STATE_STOPPING
    };

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

    Bebop(degrees_per_second_t maxYawSpeed = DefaultMaximumYawSpeed,
          meters_per_second_t maxVerticalSpeed = DefaultMaximumVerticalSpeed,
          degree_t maxTilt = DefaultMaximumTilt);
    virtual ~Bebop() override;

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
    RelativeMoveState getRelativeMoveState() const;
    std::pair<Vector3<meter_t>, radian_t> getRelativeMovePoseDifference() const;
    void resetRelativeMoveState();

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
    template<class UnitType>
    class LimitValues
    {
    public:
        UnitType m_UserMaximum;

        inline void onChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                              const char *currentKey,
                              const char *minKey,
                              const char *maxKey)
        {
            ARCONTROLLER_DICTIONARY_ELEMENT_t *elem = nullptr;
            HASH_FIND_STR(dict, ARCONTROLLER_DICTIONARY_SINGLE_KEY, elem);
            if (elem) {
                ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;

                // get current value
                HASH_FIND_STR(elem->arguments, currentKey, arg);
                if (arg) {
                    m_Current = units::make_unit<UnitType>(arg->value.Float);
                }

                // get min value
                HASH_FIND_STR(elem->arguments, minKey, arg);
                if (arg) {
                    std::get<0>(m_Limits) = units::make_unit<UnitType>(arg->value.Float);
                }

                // get max value
                HASH_FIND_STR(elem->arguments, maxKey, arg);
                if (arg) {
                    std::get<1>(m_Limits) = units::make_unit<UnitType>(arg->value.Float);
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
        }

        inline UnitType getCurrent() const
        {
            m_Semaphore.waitOnce();
            return m_Current;
        }

        inline std::pair<UnitType, UnitType> &getLimits()
        {
            m_Semaphore.waitOnce();
            return m_Limits;
        }

    private:
        UnitType m_Current;
        std::pair<UnitType, UnitType> m_Limits;
        mutable Semaphore m_Semaphore;
    };

    ControllerPtr m_Device;
    Semaphore m_StateSemaphore, m_FlatTrimSemaphore, m_BatteryLevelSemaphore;
    std::unique_ptr<VideoStream> m_VideoStream;
    FlightEventHandler m_FlightEventHandler = nullptr;
    LimitValues<degree_t> m_TiltLimits;
    LimitValues<meters_per_second_t> m_VerticalSpeedLimits;
    LimitValues<degrees_per_second_t> m_YawSpeedLimits;
    std::atomic<unsigned char> m_BatteryLevel;
    std::atomic<RelativeMoveState> m_RelativeMoveState{ RelativeMoveState::Initial };
    Vector3<meter_t> m_RelativeMovePositionDistance{ 0_m, 0_m, 0_m };
    radian_t m_RelativeMoveAngleDistance{ 0_rad };

    inline void connect();
    inline void disconnect();
    void startStreaming();
    void stopStreaming();
    inline void addEventHandlers();
    inline void onBatteryChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    inline void createControllerDevice();
    inline State getStateUpdate();

    // speed limits
    inline void setMaximumTilt(degree_t newValue);
    inline void setMaximumVerticalSpeed(meters_per_second_t newValue);
    inline void setMaximumYawSpeed(degrees_per_second_t newValue);

    static void commandReceived(eARCONTROLLER_DICTIONARY_KEY key,
                                ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                                void *data);
    void relativeMoveEnded(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    static void alertStateChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    static void productVersionReceived(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    static void magnetometerCalibrationStateReceived(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
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
