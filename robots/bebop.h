#pragma once

#ifndef DUMMY_DRONE
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
#endif // !DUMMY_DRONE

// C++ includes
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../common/semaphore.h"
#include "../hid/joystick.h"
#include "../video/input.h"

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

namespace BoBRobotics {
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
        AVFrame *m_FrameRGBPtr = nullptr;
        AVPacket m_Packet;
        SwsContext *m_ImgConvertContextPtr = nullptr;
        uint8_t *m_FrameRGBRawPtr = nullptr;
        bool m_UpdateCodecParams = false;
        std::vector<uint8_t> m_CodecData;

        static void throwOnCondition(const bool cond, const std::string &message);
        bool initCodec();
        bool reallocateBuffers();
        void cleanupBuffers();
        void reset();
        void convertFrameToRGB();
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

    Bebop();
    ~Bebop();
    void addJoystick(HID::Joystick &joystick, const float maxSpeed);
    void connect();
    void disconnect();
    void takeOff();
    void land();
    VideoStream &getVideoStream();
    void setPitch(const float pitch);
    void setRoll(const float right);
    void setAscent(const float up);
    void setYaw(const float right);
    void stopMoving();
    void takePhoto();
    void setFlightEventHandler(FlightEventHandler);

private:
    ControllerPtr m_Device;
    Semaphore m_Semaphore;
    std::unique_ptr<VideoStream> m_VideoStream;
    bool m_IsConnected = false;
    FlightEventHandler m_FlightEventHandler = nullptr;

    void startStreaming();
    void stopStreaming();
    bool onAxisEvent(HID::JAxis axis, float value, const float maxSpeed);
    bool onButtonEvent(HID::JButton button, bool pressed);

#ifndef DUMMY_DRONE
    inline void addEventHandlers();
    void batteryChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    inline void createControllerDevice();
    inline eARCONTROLLER_DEVICE_STATE getState();
    inline eARCONTROLLER_DEVICE_STATE getStateUpdate();

    static inline void checkArg(const float value);
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
#endif // !DUMMY_DRONE
};     // Bebop

/*
 * Do all initialisation (including discovery) but don't actually
 * connect to drone yet.
 */
Bebop::Bebop()
{
#ifdef DUMMY_DRONE
    std::cout << "Running in DUMMY_DRONE mode -- will not connect to drone!" << std::endl;
#else
#ifdef NO_FLY
    std::cout << "Running in NO_FLY mode -- drone will not move!" << std::endl;
#endif

    // silence annoying messages printed by library
    ARSAL_Print_SetCallback(printCallback);

    // object to interface with drone
    createControllerDevice();

    // to handle changes in state, incoming commands
    addEventHandlers();

    // initialise video stream object
    m_VideoStream = std::make_unique<VideoStream>(*this);
#endif // DUMMY_DRONE
}

/*
 * Disconnect properly from drone when object destroyed.
 */
Bebop::~Bebop()
{
    land();
    stopStreaming();
    disconnect();
}

/*
 * Try to make connection to drone.
 */
void
Bebop::connect()
{
    if (m_IsConnected) {
        return;
    }

#ifndef DUMMY_DRONE
    // send start signal
    checkError(ARCONTROLLER_Device_Start(m_Device.get()));

    // wait for start
    eARCONTROLLER_DEVICE_STATE state;
    do {
        state = getStateUpdate();
    } while (state == ARCONTROLLER_DEVICE_STATE_STARTING);

    if (state != ARCONTROLLER_DEVICE_STATE_RUNNING) {
        throw std::runtime_error("Could not connect to drone");
    }
#else
    std::cout << "Drone started" << std::endl;
#endif
    m_IsConnected = true;
}

/*
 * Try to disconnect from drone.
 */
void
Bebop::disconnect()
{
#ifndef DUMMY_DRONE
    if (m_Device) {
        // send stop signal
        checkError(ARCONTROLLER_Device_Stop(m_Device.get()));

        if (m_IsConnected) {
            // wait for stop
            eARCONTROLLER_DEVICE_STATE state;
            do {
                state = getStateUpdate();
            } while (state == ARCONTROLLER_DEVICE_STATE_STOPPING);

            if (state != ARCONTROLLER_DEVICE_STATE_STOPPED) {
                std::cerr << "Warning: Could not disconnect from drone" << std::endl;
            }
        }

        m_IsConnected = false;
    }
#endif
}

/*
 * Start controlling this drone with a joystick.
 */
void
Bebop::addJoystick(HID::Joystick &joystick, const float maxSpeed = 0.25)
{
    if (maxSpeed < 0 || maxSpeed > 1) {
        throw std::invalid_argument("maxSpeed must be between 0 and 1");
    }

    joystick.addHandler([this, maxSpeed](HID::JAxis axis, float value) {
        return onAxisEvent(axis, value, maxSpeed);
    });
    joystick.addHandler([this](HID::JButton button, bool pressed) {
        return onButtonEvent(button, pressed);
    });
}

/*
 * Send take-off command.
 */
void
Bebop::takeOff()
{
    if (m_IsConnected) {
        std::cout << "Taking off..." << std::endl;
#ifndef NO_FLY
        checkError(m_Device->aRDrone3->sendPilotingTakeOff(m_Device->aRDrone3));
#endif
        if (m_FlightEventHandler) {
            m_FlightEventHandler(true);
        }
    }
}

/*
 * Send land command.
 */
void
Bebop::land()
{
    if (m_IsConnected) {
        std::cout << "Landing..." << std::endl;
#ifndef NO_FLY
        checkError(m_Device->aRDrone3->sendPilotingLanding(m_Device->aRDrone3));
#endif
        if (m_FlightEventHandler) {
            m_FlightEventHandler(false);
        }
    }
}

Bebop::VideoStream &
Bebop::getVideoStream()
{
    startStreaming();
    return *m_VideoStream;
}

/*
 * Set drone's pitch, for moving forwards and backwards.
 */
void
Bebop::setPitch(const float pitch)
{
    checkArg(pitch);
    if (m_IsConnected) {
#ifdef NO_FLY
        std::cout << "Setting pitch to " << pitch << std::endl;
#else
        const int8_t ipitch = round(pitch * 100.0f);
        checkError(m_Device->aRDrone3->setPilotingPCMDPitch(m_Device->aRDrone3, ipitch));
        checkError(m_Device->aRDrone3->setPilotingPCMDFlag(m_Device->aRDrone3, 1));
#endif
    }
}

/*
 * Set drone's roll, for banking left and right.
 */
void
Bebop::setRoll(const float right)
{
    checkArg(right);
    if (m_IsConnected) {
#ifdef NO_FLY
        std::cout << "Setting roll to " << right << std::endl;
#else
        const int8_t iright = round(right * 100.0f);
        checkError(m_Device->aRDrone3->setPilotingPCMDRoll(m_Device->aRDrone3, iright));
        checkError(m_Device->aRDrone3->setPilotingPCMDFlag(m_Device->aRDrone3, 1));
#endif
    }
}

/*
 * Set drone's up/down motion for ascending/descending.
 */
void
Bebop::setAscent(const float up)
{
    checkArg(up);
    if (m_IsConnected) {
#ifdef NO_FLY
        std::cout << "Setting up/down to " << up << std::endl;
#else
        const int8_t iup = round(up * 100.0f);
        checkError(m_Device->aRDrone3->setPilotingPCMDGaz(m_Device->aRDrone3, iup));
#endif
    }
}

/*
 * Set drone's yaw. The drone will turn really slowly.
 */
void
Bebop::setYaw(const float right)
{
    checkArg(right);
    if (m_IsConnected) {
#ifdef NO_FLY
        std::cout << "Setting yaw to " << right << std::endl;
#else
        const int8_t iright = round(right * 100.0f);
        checkError(m_Device->aRDrone3->setPilotingPCMDYaw(m_Device->aRDrone3, iright));
#endif
    }
}

/*
 * Send the command to start video streaming.
 */
void
Bebop::startStreaming()
{
    checkError(m_Device->aRDrone3->sendMediaStreamingVideoEnable(m_Device->aRDrone3, 1));
}

/*
 * Send the command to stop video streaming.
 */
void
Bebop::stopStreaming()
{
    checkError(m_Device->aRDrone3->sendMediaStreamingVideoEnable(m_Device->aRDrone3, 0));
}

/*
 * Stops the drone from moving along all axes.
 */
void
Bebop::stopMoving()
{
    if (m_IsConnected) {
        setPitch(0);
        setRoll(0);
        setYaw(0);
        setAscent(0);
    }
}

/*
 * Tells the drone to take a photo and store it.
 */
void
Bebop::takePhoto()
{
    if (m_IsConnected) {
        std::cout << "Taking photo" << std::endl;
#ifndef DUMMY_DRONE
        checkError(m_Device->aRDrone3->sendMediaRecordPicture(m_Device->aRDrone3, 0));
#endif
    }
}

/*
 * Adds an event handler for when the drone is taking off or landing, indicated
 * by its parameter.
 */
void
Bebop::setFlightEventHandler(FlightEventHandler handler)
{
    m_FlightEventHandler = handler;
}

#ifndef DUMMY_DRONE
/*
 * Waits for the drone to send a state-update command and returns the new
 * state.
 */
inline eARCONTROLLER_DEVICE_STATE
Bebop::getStateUpdate()
{
    m_Semaphore.wait();
    return getState();
}

/*
 * Returns the drone's connection state.
 */
inline eARCONTROLLER_DEVICE_STATE
Bebop::getState()
{
    auto err = ARCONTROLLER_OK;
    const auto state = ARCONTROLLER_Device_GetState(m_Device.get(), &err);
    checkError(err);
    return state;
}

/*
 * Create the struct used by the ARSDK to interface with the drone.
 */
inline void
Bebop::createControllerDevice()
{
    // create discovery device
    static const auto deleter = [](ARDISCOVERY_Device_t *discover) {
        ARDISCOVERY_Device_Delete(&discover);
    };
    auto derr = ARDISCOVERY_OK;
    const std::unique_ptr<ARDISCOVERY_Device_t, decltype(deleter)> discover(ARDISCOVERY_Device_New(&derr), deleter);
    checkError(derr);

    // try to discover device on network
    checkError(ARDISCOVERY_Device_InitWifi(discover.get(),
                                           ARDISCOVERY_PRODUCT_BEBOP_2,
                                           "bebop",
                                           BEBOP_IP_ADDRESS,
                                           BEBOP_DISCOVERY_PORT));

    // create controller object
    auto err = ARCONTROLLER_OK;
    m_Device = ControllerPtr(ARCONTROLLER_Device_New(discover.get(), &err),
                             [](ARCONTROLLER_Device_t *dev) {
                                 ARCONTROLLER_Device_Delete(&dev);
                             });
    checkError(err);
}

/*
 * Add callbacks for when connection state changes or a command is received
 * from the drone.
 */
inline void
Bebop::addEventHandlers()
{
    checkError(ARCONTROLLER_Device_AddStateChangedCallback(
            m_Device.get(), stateChanged, this));
    checkError(ARCONTROLLER_Device_AddCommandReceivedCallback(
            m_Device.get(), commandReceived, this));
}

/*
 * Invoked by commandReceived().
 *
 * Prints the battery state whenever it changes.
 */
inline void
Bebop::batteryChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
{
    // which command was received?
    ARCONTROLLER_DICTIONARY_ELEMENT_t *elem = nullptr;
    HASH_FIND_STR(dict, ARCONTROLLER_DICTIONARY_SINGLE_KEY, elem);
    if (!elem) {
        return;
    }

    // get associated value
    ARCONTROLLER_DICTIONARY_ARG_t *val = nullptr;
    HASH_FIND_STR(
            elem->arguments,
            ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT,
            val);

    if (val) {
        // print battery status
        std::cout << "Battery: " << (int) val->value.U8 << "%" << std::endl;
    }
}

/*
 * Empty function used to suppress default ARSDK console messages.
 */
int
Bebop::printCallback(eARSAL_PRINT_LEVEL level,
                     const char *tag,
                     const char *format,
                     va_list va)
{
    // do nothing
    return 0;
}

/*
 * Invoked when the drone's connection state changes.
 */
void
Bebop::stateChanged(eARCONTROLLER_DEVICE_STATE newstate,
                    eARCONTROLLER_ERROR err,
                    void *data)
{
    Bebop *bebop = reinterpret_cast<Bebop *>(data);
    bebop->m_Semaphore.notify(); // trigger semaphore used by get_state_update()

    switch (newstate) {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
        std::cout << "Drone stopped" << std::endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_STARTING:
        std::cout << "Drone starting..." << std::endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_RUNNING:
        std::cout << "Drone started" << std::endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_PAUSED:
        std::cout << "Drone is paused" << std::endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_STOPPING:
        std::cout << "Drone stopping..." << std::endl;
        break;
    default:
        std::cerr << "Unknown state!" << std::endl;
    }
}

/*
 * Invoked when a command is received from drone.
 */
void
Bebop::commandReceived(eARCONTROLLER_DICTIONARY_KEY key,
                       ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                       void *data)
{
    if (!dict) {
        return;
    }

    if (key == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED) {
        Bebop *bebop = reinterpret_cast<Bebop *>(data);
        bebop->batteryChanged(dict);
    }
}

bool
Bebop::onAxisEvent(HID::JAxis axis, float value, const float maxSpeed)
{
    /* 
     * setRoll/Pitch etc. all take values between -1 and 1. We cap these 
     * values for the joystick code to make the drone more controllable. 
     */
    switch (axis) {
    case HID::JAxis::RightStickHorizontal:
        setRoll(maxSpeed * value);
        return true;
    case HID::JAxis::RightStickVertical:
        setPitch(maxSpeed * -value);
        return true;
    case HID::JAxis::LeftStickVertical:
        setAscent(maxSpeed * -value);
        return true;
    case HID::JAxis::LeftTrigger:
        setYaw(maxSpeed * -value);
        return true;
    case HID::JAxis::RightTrigger:
        setYaw(maxSpeed * value);
        return true;
    default:
        // otherwise signal that we haven't handled event
        return false;
    }
}

bool
Bebop::onButtonEvent(HID::JButton button, bool pressed)
{
    // we only care about button presses
    if (!pressed) {
        return false;
    }

    // A = take off; B = land
    switch (button) {
    case HID::JButton::A:
        takeOff();
        return true;
    case HID::JButton::B:
        land();
        return true;
    default:
        // otherwise signal that we haven't handled event
        return false;
    }
}

void
Bebop::checkArg(const float value)
{
    if (value > 1.0f || value < -1.0f) {
        throw std::invalid_argument("Argument must be between -1.0 and 1.0");
    }
}

void
Bebop::VideoStream::throwOnCondition(const bool cond, const std::string &message)
{
    if (!cond)
        return;
    throw std::runtime_error(message);
}

bool
Bebop::VideoStream::initCodec()
{
    if (m_CodecInitialised) {
        return true;
    }

    try {
        // Very first init
        avcodec_register_all();
        av_register_all();
        av_log_set_level(AV_LOG_QUIET);

        m_CodecPtr = avcodec_find_decoder(AV_CODEC_ID_H264);
        throwOnCondition(m_CodecPtr == nullptr, "Codec H264 not found!");

        m_CodecContextPtr = avcodec_alloc_context3(m_CodecPtr);
        m_CodecContextPtr->pix_fmt = AV_PIX_FMT_YUV420P;
        m_CodecContextPtr->skip_frame = AVDISCARD_DEFAULT;
        m_CodecContextPtr->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
        m_CodecContextPtr->skip_loop_filter = AVDISCARD_DEFAULT;
        m_CodecContextPtr->workaround_bugs = AVMEDIA_TYPE_VIDEO;
        m_CodecContextPtr->codec_id = AV_CODEC_ID_H264;
        m_CodecContextPtr->skip_idct = AVDISCARD_DEFAULT;
        // At the beginning we have no idea about the frame size
        m_CodecContextPtr->width = 0;
        m_CodecContextPtr->height = 0;

        if (m_CodecPtr->capabilities & CODEC_CAP_TRUNCATED) {
            m_CodecContextPtr->flags |= CODEC_FLAG_TRUNCATED;
        }
        m_CodecContextPtr->flags2 |= CODEC_FLAG2_CHUNKS;

        m_FramePtr = av_frame_alloc();
        throwOnCondition(!m_FramePtr, "Can not allocate memory for frames!");

        throwOnCondition(avcodec_open2(m_CodecContextPtr, m_CodecPtr, nullptr) < 0,
                         "Can not open the decoder!");

        av_init_packet(&m_Packet);
    } catch (const std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        reset();
        return false;
    }

    m_CodecInitialised = true;
    m_FirstFrameReceived = false;
    std::cout << "H264 Codec is partially initialized!" << std::endl;
    return true;
}

bool
Bebop::VideoStream::reallocateBuffers()
{
    std::cout << "Buffer reallocation request" << std::endl;
    if (!m_CodecInitialised) {
        return false;
    }

    try {
        throwOnCondition(m_CodecContextPtr->width == 0 ||
                                 m_CodecContextPtr->width == 0,
                         std::string("Invalid frame size:") +
                                 std::to_string(m_CodecContextPtr->width) + " x " +
                                 std::to_string(m_CodecContextPtr->width));

        const uint32_t num_bytes = avpicture_get_size(
                AV_PIX_FMT_RGB24, m_CodecContextPtr->width, m_CodecContextPtr->width);
        m_FrameRGBPtr = av_frame_alloc();

        throwOnCondition(!m_FrameRGBPtr,
                         "Can not allocate memory for frames!");

        m_FrameRGBRawPtr =
                reinterpret_cast<uint8_t *>(av_malloc(num_bytes * sizeof(uint8_t)));
        throwOnCondition(
                m_FrameRGBRawPtr == nullptr,
                std::string("Can not allocate memory for the buffer: ") +
                        std::to_string(num_bytes));
        throwOnCondition(0 == avpicture_fill(reinterpret_cast<AVPicture *>(
                                                     m_FrameRGBPtr),
                                             m_FrameRGBRawPtr,
                                             AV_PIX_FMT_RGB24,
                                             m_CodecContextPtr->width,
                                             m_CodecContextPtr->height),
                         "Failed to initialize the picture data structure.");

        m_ImgConvertContextPtr = sws_getContext(m_CodecContextPtr->width,
                                                m_CodecContextPtr->height,
                                                m_CodecContextPtr->pix_fmt,
                                                m_CodecContextPtr->width,
                                                m_CodecContextPtr->height,
                                                AV_PIX_FMT_RGB24,
                                                SWS_FAST_BILINEAR,
                                                nullptr,
                                                nullptr,
                                                nullptr);
    } catch (const std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        reset(); // reset() is intentional
        return false;
    }

    return true;
}

void
Bebop::VideoStream::cleanupBuffers()
{
    if (m_FrameRGBPtr) {
        av_free(m_FrameRGBPtr);
    }

    if (m_FrameRGBRawPtr) {
        av_free(m_FrameRGBRawPtr);
    }

    if (m_ImgConvertContextPtr) {
        sws_freeContext(m_ImgConvertContextPtr);
    }
}

void
Bebop::VideoStream::reset()
{
    if (m_CodecContextPtr) {
        avcodec_close(m_CodecContextPtr);
    }

    if (m_FramePtr) {
        av_free(m_FramePtr);
    }

    cleanupBuffers();

    m_CodecInitialised = false;
    m_FirstFrameReceived = false;
}

Bebop::VideoStream::VideoStream(Bebop &bebop)
{
    checkError(ARCONTROLLER_Device_SetVideoStreamCallbacks(
        bebop.m_Device.get(), configCallback, frameCallback, nullptr, this));
}

Bebop::VideoStream::~VideoStream()
{
    reset();
}

void
Bebop::VideoStream::convertFrameToRGB()
{
    if (!m_CodecContextPtr->width || !m_CodecContextPtr->height)
        return;
    sws_scale(m_ImgConvertContextPtr,
              m_FramePtr->data,
              m_FramePtr->linesize,
              0,
              m_CodecContextPtr->height,
              m_FrameRGBPtr->data,
              m_FrameRGBPtr->linesize);
}

bool
Bebop::VideoStream::setH264Params(uint8_t *sps_buffer_ptr,
                                  uint32_t sps_buffer_size,
                                  uint8_t *pps_buffer_ptr,
                                  uint32_t pps_buffer_size)
{
    // This function is called in the same thread as decode(), so no sync is
    // necessary
    // TODO: Exact sizes + more error checkings
    m_UpdateCodecParams = (sps_buffer_ptr && pps_buffer_ptr &&
                           sps_buffer_size && pps_buffer_size &&
                           (pps_buffer_size < 32) && (sps_buffer_size < 32));

    if (m_UpdateCodecParams) {
        m_CodecData.resize(sps_buffer_size + pps_buffer_size);
        std::copy(sps_buffer_ptr,
                  sps_buffer_ptr + sps_buffer_size,
                  m_CodecData.begin());
        std::copy(pps_buffer_ptr,
                  pps_buffer_ptr + pps_buffer_size,
                  m_CodecData.begin() + sps_buffer_size);
    } else {
        // invalid data
        m_CodecData.clear();
    }

    return m_UpdateCodecParams;
}

bool
Bebop::VideoStream::decode(const ARCONTROLLER_Frame_t *framePtr)
{
    if (!m_CodecInitialised) {
        if (!initCodec()) {
            std::cerr << "Codec initialization failed!" << std::endl;
            return false;
        }
    }

    /*
     * For VideoStream2, we trick avcodec whenever we receive a new SPS/PPS
     * info from the Bebop. setH264Params() function will fill a buffer with
     * SPS/PPS data, then these are passed to avcodec_decode_video2() here, once
     * for each SPS/PPS update. Apparantly, avcodec_decode_video2() function
     * picks up the changes and apply them to upcoming video packets.
     *
     * More info on VS v2.0:
     * http://developer.parrot.com/blog/2016/ARSDK-3-8-release/
     *
     * */
    if (m_UpdateCodecParams && m_CodecData.size()) {
        std::cout << "Updating H264 codec parameters (Buffer Size: "
                  << m_CodecData.size() << ") ..." << std::endl;
        m_Packet.data = &m_CodecData[0];
        m_Packet.size = m_CodecData.size();
        int32_t frame_finished = 0;
        const int32_t len = avcodec_decode_video2(
                m_CodecContextPtr, m_FramePtr, &frame_finished, &m_Packet);
        if (len >= 0 && len == m_Packet.size) {
            // success, skip this step until next codec update
            m_UpdateCodecParams = false;
        } else {
            std::cerr << "Unexpected error while updating H264 parameters."
                      << std::endl;
            return false;
        }
    }

    if (!framePtr->data || !framePtr->used) {
        std::cerr << "Invalid frame data. Skipping." << std::endl;
        return false;
    }

    m_Packet.data = framePtr->data;
    m_Packet.size = framePtr->used;

    const uint32_t width_prev = getFrameWidth();
    const uint32_t height_prev = getFrameHeight();

    int32_t frame_finished = 0;
    while (m_Packet.size > 0) {
        const int32_t len = avcodec_decode_video2(
                m_CodecContextPtr, m_FramePtr, &frame_finished, &m_Packet);
        if (len >= 0) {
            if (frame_finished) {
                if ((getFrameWidth() != width_prev) ||
                    (getFrameHeight() != height_prev)) {
                    std::cerr << "Frame size changed to "
                              << getFrameWidth() << " x "
                              << getFrameHeight() << std::endl;
                    if (!reallocateBuffers()) {
                        std::cerr << "Buffer reallocation failed!" << std::endl;
                    }
                }
                convertFrameToRGB();
            }

            if (m_Packet.data) {
                m_Packet.size -= len;
                m_Packet.data += len;
            }
        } else {
            return false;
        }
    }
    return true;
} // class VideoDecoder
#endif

// start VideoStream class
cv::Size
Bebop::VideoStream::getOutputSize() const
{
    return cv::Size(VIDEO_WIDTH, VIDEO_HEIGHT);
}

bool
Bebop::VideoStream::readFrame(cv::Mat &frame)
{
    std::lock_guard<decltype(m_FrameMutex)> guard(m_FrameMutex);
    if (!m_NewFrame) {
        return false;
    }

    m_Frame.copyTo(frame);
    return true;
}

eARCONTROLLER_ERROR
Bebop::VideoStream::configCallback(ARCONTROLLER_Stream_Codec_t codec, void *data)
{
    auto stream = reinterpret_cast<VideoStream *>(data);
    stream->setH264Params(codec.parameters.h264parameters.spsBuffer,
                          codec.parameters.h264parameters.spsSize,
                          codec.parameters.h264parameters.ppsBuffer,
                          codec.parameters.h264parameters.ppsSize);
    return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR
Bebop::VideoStream::frameCallback(ARCONTROLLER_Frame_t *frame, void *data)
{
    auto stream = reinterpret_cast<VideoStream *>(data);
    if (stream->decode(frame)) {
        // convert into BGR cv::Mat
        std::lock_guard<decltype(stream->m_FrameMutex)> guard(stream->m_FrameMutex);
        cv::Mat frameRGB(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3, (void *) stream->m_FrameRGBRawPtr);
        cv::cvtColor(frameRGB, stream->m_Frame, CV_RGB2BGR, 3);
        stream->m_NewFrame = true;
    }
    return ARCONTROLLER_OK;
}
// end VideoStream class
} // Robots
} // GeNNRobotics
