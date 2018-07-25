#pragma once

// Standard C++ includes
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

// OpenCV
#ifdef KEY_UP
#undef KEY_UP
#endif
#ifdef KEY_DOWN
#undef KEY_DOWN
#endif
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../common/semaphore.h"
#include "../hid/joystick.h"
#include "../video/input.h"

// Third-party includes
#include "../third_party/units.h"

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
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::velocity;

using FlightEventHandler = std::function<void(bool takeoff)>;

template<class T>
using Limits = std::tuple<T, T>;

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

/*
 * Main class for interfacing with drone. Handles connection/disconnection and
 * sending steering commands.
 *
 * Video stream functionality is implemented in video.h by BebopVideoStream
 * class.
 */
class Bebop
{
    using ControllerPtr = std::unique_ptr<ARCONTROLLER_Device_t, std::function<void(ARCONTROLLER_Device_t *)>>;

public:
    class VideoStream : public Video::Input
    {
    public:
        VideoStream(Bebop &bebop);
        ~VideoStream();
        virtual bool readFrame(cv::Mat &) override;
        virtual cv::Size getOutputSize() const override;

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
        inline uint32_t getFrameWidth() const
        {
            return m_CodecInitialised ? m_CodecContextPtr->width : 0;
        }
        inline uint32_t getFrameHeight() const
        {
            return m_CodecInitialised ? m_CodecContextPtr->height : 0;
        }

        static eARCONTROLLER_ERROR configCallback(ARCONTROLLER_Stream_Codec_t codec, void *data);
        static eARCONTROLLER_ERROR frameCallback(ARCONTROLLER_Frame_t *frame, void *data);
    }; // VideoStream

    Bebop(degrees_per_second_t maxYawSpeed = DefaultMaximumYawSpeed,
          meters_per_second_t maxVerticalSpeed = DefaultMaximumVerticalSpeed,
          degree_t maxTilt = DefaultMaximumTilt);
    ~Bebop();
    void addJoystick(HID::Joystick &joystick);

    // speed limits
    degree_t getMaximumTilt() const;
    Limits<degree_t> &getTiltLimits();
    meters_per_second_t getMaximumVerticalSpeed() const;
    Limits<meters_per_second_t> &getVerticalSpeedLimits();
    degrees_per_second_t getMaximumYawSpeed() const;
    Limits<degrees_per_second_t> &getYawSpeedLimits();

    // motor control
    void takeOff();
    void land();
    void setPitch(float pitch);
    void setRoll(float right);
    void setVerticalSpeed(float up);
    void setYawSpeed(float right);
    void stopMoving();

    // misc
    VideoStream &getVideoStream();
    void takePhoto();
    void setFlightEventHandler(FlightEventHandler);

    // defaults
    static constexpr auto DefaultMaximumTilt = 8_deg;
    static constexpr auto DefaultMaximumYawSpeed = 100_deg_per_s;
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

                // std::cout << "Max: " << m_Current << " ("
                //           << std::get<0>(m_Limits) << ", "
                //           << std::get<1>(m_Limits) << ")" << std::endl;
            }
        }

        inline UnitType getCurrent() const
        {
            m_Semaphore.waitOnce();
            return m_Current;
        }

        inline Limits<UnitType> &getLimits()
        {
            m_Semaphore.waitOnce();
            return m_Limits;
        }

    private:
        UnitType m_Current;
        Limits<UnitType> m_Limits;
        mutable Semaphore m_Semaphore;
    };

    ControllerPtr m_Device;
    Semaphore m_Semaphore;
    std::unique_ptr<VideoStream> m_VideoStream;
    FlightEventHandler m_FlightEventHandler = nullptr;
    LimitValues<degree_t> m_TiltLimits;
    LimitValues<meters_per_second_t> m_VerticalSpeedLimits;
    LimitValues<degrees_per_second_t> m_YawSpeedLimits;

    inline void connect();
    inline void disconnect();
    void startStreaming();
    void stopStreaming();
    bool onAxisEvent(HID::JAxis axis, float value);
    bool onButtonEvent(HID::JButton button, bool pressed);
    inline void addEventHandlers();
    inline void onBatteryChanged(const ARCONTROLLER_DICTIONARY_ELEMENT_t *dict) const;
    inline void createControllerDevice();
    inline eARCONTROLLER_DEVICE_STATE getState();
    inline eARCONTROLLER_DEVICE_STATE getStateUpdate();

    // speed limits
    inline void setMaximumTilt(degree_t newValue);
    inline void setMaximumVerticalSpeed(meters_per_second_t newValue);
    inline void setMaximumYawSpeed(degrees_per_second_t newValue);

    static void commandReceived(eARCONTROLLER_DICTIONARY_KEY key,
                                ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                                void *data);
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