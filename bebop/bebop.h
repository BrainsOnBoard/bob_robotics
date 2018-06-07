#pragma once

#ifndef DUMMY_DRONE
extern "C"
{
// ARSDK includes
#include "libARController/ARCONTROLLER_Error.h"
#include "libARController/ARCONTROLLER_Frame.h"
#include "libARSAL/ARSAL_Print.h"
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>

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
#endif // !DUMMY_DRONE

// C++ includes
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

// GeNN robotics includes
#include "../common/semaphore.h"
#include "../hid/joystick.h"

// POSIX includes
#include <signal.h>

// these values are hardcoded for Bebop drones
#define BEBOP_IP_ADDRESS "192.168.42.1"
#define BEBOP_DISCOVERY_PORT 44444

/*
 * If the DUMMY_DRONE flag is set, we don't connect to drone and just output
 * messages to console when steering commands are sent.
 *
 * If the NO_FLY flag is set, we *do* connect to the drone, but don't let it
 * fly.
 */
#ifdef DUMMY_DRONE
#define NO_FLY
#endif

namespace GeNNRobotics {
namespace Robots {
/*
 * Simply throws a runtime_error with appropriate message if
 * err != ARCONTROLLER_OK.
 */
void
checkError(eARCONTROLLER_ERROR err)
{
    if (err != ARCONTROLLER_OK) {
        throw std::runtime_error(std::string("Controller error: ") +
                                 ARCONTROLLER_Error_ToString(err));
    }
}

void
checkError(eARDISCOVERY_ERROR err)
{
    if (err != ARDISCOVERY_OK) {
        throw std::runtime_error(std::string("Discovery error: ") +
                                 ARDISCOVERY_Error_ToString(err));
    }
}

using FlightEventHandler = std::function<void(bool takeoff)>;

#define VIDEO_WIDTH 856
#define VIDEO_HEIGHT 480

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

    class VideoStream
    {
        using UserVideoCallback = std::function<void(const uint8_t *frame, void *userdata)>;
        class VideoDecoder
        {
        private:
            static const char *LOG_TAG;

            bool codec_initialized_;
            bool first_iframe_recv_;
            AVFormatContext *format_ctx_ptr_;
            AVCodecContext *codec_ctx_ptr_;
            AVCodec *codec_ptr_;
            AVFrame *frame_ptr_;
            AVFrame *frame_rgb_ptr_;
            AVPacket packet_;
            SwsContext *img_convert_ctx_ptr_;
            AVInputFormat *input_format_ptr_;
            uint8_t *frame_rgb_raw_ptr_;

            bool update_codec_params_;
            std::vector<uint8_t> codec_data_;

            static void ThrowOnCondition(const bool cond, const std::string &message);
            bool InitCodec();
            bool ReallocateBuffers();
            void CleanupBuffers();
            void Reset();

            void ConvertFrameToRGB();

        public:
            VideoDecoder();
            ~VideoDecoder();

            bool SetH264Params(uint8_t *sps_buffer_ptr,
                               uint32_t sps_buffer_size,
                               uint8_t *pps_buffer_ptr,
                               uint32_t pps_buffer_size);
            bool Decode(const ARCONTROLLER_Frame_t *bebop_frame_ptr_);
            inline uint32_t GetFrameWidth() const
            {
                return codec_initialized_ ? codec_ctx_ptr_->width : 0;
            }
            inline uint32_t GetFrameHeight() const
            {
                return codec_initialized_ ? codec_ctx_ptr_->height : 0;
            }

            inline const uint8_t *GetFrameRGBRawCstPtr() const
            {
                return frame_rgb_raw_ptr_;
            }
        };

    public:
        VideoStream(Bebop *bebop);
        ~VideoStream();
        void startStreaming();
        void startStreaming(UserVideoCallback cb, void *userdata);
        void stopStreaming();

    private:
        ARCONTROLLER_Device_t *m_Device;
        std::unique_ptr<VideoDecoder> m_Decoder;
        UserVideoCallback m_UserCallback = nullptr;
        void *m_UserVideoCallbackData;

        /*
         * Invoked when we receive a packet containing H264 params.
         */
        static eARCONTROLLER_ERROR configCallback(ARCONTROLLER_Stream_Codec_t codec,
                                                  void *data);

        /*
         * Invoked when we receive a packet containing an H264-encoded frame.
         */
        static eARCONTROLLER_ERROR frameCallback(ARCONTROLLER_Frame_t *frame,
                                                 void *data);
    }; // VideoStream
private:
    Semaphore m_Semaphore;
    bool m_IsConnected = false;
    FlightEventHandler m_FlightEventHandler = nullptr;
    static constexpr float MaxBank = 50; // maximum % of speed for pitch/rool
    static constexpr float MaxUp = 50;   // maximum % of speed for up/down motion
    static constexpr float MaxYaw = 100; // maximum % of speed for yaw

    bool onAxisEvent(HID::JAxis axis, float value);
    bool onButtonEvent(HID::JButton button, bool pressed);

#ifndef DUMMY_DRONE
    inline eARCONTROLLER_DEVICE_STATE getStateUpdate();
    inline eARCONTROLLER_DEVICE_STATE getState();
    inline void createControllerDevice();
    inline void addEventHandlers();
    void batteryChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    static int printCallback(eARSAL_PRINT_LEVEL level,
                             const char *tag,
                             const char *format,
                             va_list va);
    static void stateChanged(eARCONTROLLER_DEVICE_STATE newstate,
                             eARCONTROLLER_ERROR err,
                             void *data);
    static void commandReceived(eARCONTROLLER_DICTIONARY_KEY key,
                                ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                                void *data);
#endif // !DUMMY_DRONE

public:
    ControllerPtr m_Device;

    Bebop();
    ~Bebop();
    void addJoystick(HID::Joystick &joystick);
    void connect();
    void disconnect();
    void takeOff();
    void land();
    void setPitch(int8_t pitch);
    void setRoll(int8_t right);
    void setUpDown(int8_t up);
    void setYaw(int8_t right);
    void stopMoving();
    void takePhoto();
    void setFlightEventHandler(FlightEventHandler);
}; // Bebop
} // Robots
} // GeNNRobotics
