#include "bebop.h"

#define DRONE_COMMAND_NO_ARG(COMMAND) \
    checkError(m_Device->aRDrone3->COMMAND(m_Device->aRDrone3));

#define DRONE_COMMAND(COMMAND, ARG) \
    checkError(m_Device->aRDrone3->COMMAND(m_Device->aRDrone3, ARG));

#define MAX_SPEED_CHANGED(NAME, KEY)                                         \
    {                                                                        \
        bebop->m_##NAME##Limits.onChanged(                                   \
                dict,                                                        \
                ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_##KEY##CHANGED_CURRENT, \
                ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_##KEY##CHANGED_MIN,     \
                ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_##KEY##CHANGED_MAX);    \
    }

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

/*
 * Throws a runtime_error if discovery device fails.
 */
void
checkError(eARDISCOVERY_ERROR err)
{
    if (err != ARDISCOVERY_OK) {
        throw std::runtime_error(std::string("Discovery error: ") +
                                 ARDISCOVERY_Error_ToString(err));
    }
}

namespace BoBRobotics {
namespace Robots {

// We also have to give definitions for these variables, because c++ is weird
constexpr degree_t Bebop::DefaultMaximumTilt;
constexpr degrees_per_second_t Bebop::DefaultMaximumYawSpeed;
constexpr meters_per_second_t Bebop::DefaultMaximumVerticalSpeed;

/*
 * Do all initialisation (including discovery) but don't actually
 * connect to drone yet.
 */
Bebop::Bebop(degrees_per_second_t maxYawSpeed,
             meters_per_second_t maxVerticalSpeed,
             degree_t maxTilt)
{
    // silence annoying messages printed by library
    ARSAL_Print_SetCallback(printCallback);

    // object to interface with drone
    createControllerDevice();

    // to handle changes in state, incoming commands
    addEventHandlers();

    // initialise video stream object
    m_VideoStream = std::make_unique<VideoStream>(*this);

    // store speed limits for later
    m_YawSpeedLimits.m_UserMaximum = maxYawSpeed;
    m_VerticalSpeedLimits.m_UserMaximum = maxVerticalSpeed;
    m_TiltLimits.m_UserMaximum = maxTilt;

    // connect to drone
    connect();
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

    // set speed limits
    setMaximumYawSpeed(m_YawSpeedLimits.m_UserMaximum);
    setMaximumVerticalSpeed(m_VerticalSpeedLimits.m_UserMaximum);
    setMaximumTilt(m_TiltLimits.m_UserMaximum);
}

/*
 * Try to disconnect from drone.
 */
void
Bebop::disconnect()
{
    if (m_Device) {
        // send stop signal
        checkError(ARCONTROLLER_Device_Stop(m_Device.get()));

        // wait for stop
        eARCONTROLLER_DEVICE_STATE state;
        do {
            state = getStateUpdate();
        } while (state == ARCONTROLLER_DEVICE_STATE_STOPPING);

        if (state != ARCONTROLLER_DEVICE_STATE_STOPPED) {
            std::cerr << "Warning: Could not disconnect from drone" << std::endl;
        }
    }
}

/*
 * Start controlling this drone with a joystick.
 */
void
Bebop::addJoystick(HID::Joystick &joystick)
{
    joystick.addHandler([this](HID::JAxis axis, float value) {
        return onAxisEvent(axis, value);
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
    std::cout << "Taking off..." << std::endl;
    DRONE_COMMAND_NO_ARG(sendPilotingTakeOff);

    if (m_FlightEventHandler) {
        m_FlightEventHandler(true);
    }
}

/*
 * Send land command.
 */
void
Bebop::land()
{
    std::cout << "Landing..." << std::endl;
    DRONE_COMMAND_NO_ARG(sendPilotingLanding);

    if (m_FlightEventHandler) {
        m_FlightEventHandler(false);
    }
}

Bebop::VideoStream &
Bebop::getVideoStream()
{
    startStreaming();
    return *m_VideoStream;
}

degree_t
Bebop::getMaximumTilt()
{
    return m_TiltLimits.getCurrent();
}

Limits<degree_t> &
Bebop::getTiltLimits()
{
    return m_TiltLimits.getLimits();
}

meters_per_second_t
Bebop::getMaximumVerticalSpeed()
{
    return m_VerticalSpeedLimits.getCurrent();
}

Limits<meters_per_second_t> &
Bebop::getVerticalSpeedLimits()
{
    return m_VerticalSpeedLimits.getLimits();
}

degrees_per_second_t
Bebop::getMaximumYawSpeed()
{
    return m_YawSpeedLimits.getCurrent();
}

Limits<degrees_per_second_t> &
Bebop::getYawSpeedLimits()
{
    return m_YawSpeedLimits.getLimits();
}

inline void
Bebop::setMaximumTilt(degree_t newValue)
{
    DRONE_COMMAND(sendPilotingSettingsMaxTilt, newValue.value());
}

inline void
Bebop::setMaximumVerticalSpeed(meters_per_second_t newValue)
{
    DRONE_COMMAND(sendSpeedSettingsMaxVerticalSpeed, newValue.value());
}

inline void
Bebop::setMaximumYawSpeed(degrees_per_second_t newValue)
{
    DRONE_COMMAND(sendSpeedSettingsMaxRotationSpeed, newValue.value());
}

/*
 * Set drone's pitch, for moving forwards and backwards.
 */
void
Bebop::setPitch(float pitch)
{
    assert(pitch >= -1.0f && pitch <= 1.0f);
    DRONE_COMMAND(setPilotingPCMDPitch, round(pitch * 100.0f));
    DRONE_COMMAND(setPilotingPCMDFlag, 1);
}

/*
 * Set drone's roll, for banking left and right.
 */
void
Bebop::setRoll(float right)
{
    assert(right >= -1.0f && right <= 1.0f);
    DRONE_COMMAND(setPilotingPCMDRoll, round(right * 100.0f));
    DRONE_COMMAND(setPilotingPCMDFlag, 1);
}

/*
 * Set drone's up/down motion for ascending/descending.
 */
void
Bebop::setVerticalSpeed(float up)
{
    assert(up >= -1.0f && up <= 1.0f);
    DRONE_COMMAND(setPilotingPCMDGaz, round(up * 100.0f));
}

/*
 * Set drone's yaw. The drone will turn really slowly.
 */
void
Bebop::setYawSpeed(float right)
{
    assert(right >= -1.0f && right <= 1.0f);
    DRONE_COMMAND(setPilotingPCMDYaw, round(right * 100.0f));
}

/*
 * Send the command to start video streaming.
 */
void
Bebop::startStreaming()
{
    DRONE_COMMAND(sendMediaStreamingVideoEnable, 1);
}

/*
 * Send the command to stop video streaming.
 */
void
Bebop::stopStreaming()
{
    DRONE_COMMAND(sendMediaStreamingVideoEnable, 0);
}

/*
 * Stops the drone from moving along all axes.
 */
void
Bebop::stopMoving()
{
    setPitch(0);
    setRoll(0);
    setYawSpeed(0);
    setVerticalSpeed(0);
}

/*
 * Tells the drone to take a photo and store it.
 */
void
Bebop::takePhoto()
{
    DRONE_COMMAND(sendMediaRecordPicture, 0);
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
Bebop::onBatteryChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
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

    Bebop *bebop = reinterpret_cast<Bebop *>(data);
    switch (key) {
    case ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED:
        bebop->onBatteryChanged(dict);
        break;
    case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSETTINGSSTATE_MAXTILTCHANGED:
        MAX_SPEED_CHANGED(Tilt, PILOTINGSETTINGSSTATE_MAXTILT);
        break;
    case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXROTATIONSPEEDCHANGED:
        MAX_SPEED_CHANGED(YawSpeed, SPEEDSETTINGSSTATE_MAXROTATIONSPEED);
        break;
    case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_SPEEDSETTINGSSTATE_MAXVERTICALSPEEDCHANGED:
        MAX_SPEED_CHANGED(VerticalSpeed, SPEEDSETTINGSSTATE_MAXVERTICALSPEED);
        break;
    default:
        break;
    }
}

bool
Bebop::onAxisEvent(HID::JAxis axis, float value)
{
    /* 
     * setRoll/Pitch etc. all take values between -1 and 1. We cap these 
     * values for the joystick code to make the drone more controllable. 
     */
    switch (axis) {
    case HID::JAxis::RightStickHorizontal:
        setRoll(value);
        return true;
    case HID::JAxis::RightStickVertical:
        setPitch(-value);
        return true;
    case HID::JAxis::LeftStickVertical:
        setVerticalSpeed(-value);
        return true;
    case HID::JAxis::LeftTrigger:
        setYawSpeed(-value);
        return true;
    case HID::JAxis::RightTrigger:
        setYawSpeed(value);
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
        if (!m_CodecPtr) {
            throw std::runtime_error("Codec H264 not found!");
        }

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
        if (!m_FramePtr) {
            throw std::runtime_error("Cannot allocate memory for frames!");
        }

        if (avcodec_open2(m_CodecContextPtr, m_CodecPtr, nullptr) < 0) {
            throw std::runtime_error("Cannot open the decoder!");
        }

        av_init_packet(&m_Packet);
    } catch (const std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        reset();
        return false;
    }

    m_CodecInitialised = true;
    m_FirstFrameReceived = false;
    return true;
}

bool
Bebop::VideoStream::reallocateBuffers()
{
    if (!m_CodecInitialised) {
        return false;
    }

    try {
        if (m_CodecContextPtr->width == 0 || m_CodecContextPtr->width == 0) {
            throw std::runtime_error(std::string("Invalid frame size: ") +
                                     std::to_string(m_CodecContextPtr->width) + " x " +
                                     std::to_string(m_CodecContextPtr->width));
        }

        const uint32_t num_bytes = avpicture_get_size(
                AV_PIX_FMT_BGR24, m_CodecContextPtr->width, m_CodecContextPtr->width);
        m_FrameBGRPtr = av_frame_alloc();

        if (!m_FrameBGRPtr) {
            throw std::runtime_error("Cannot allocate memory for frames!");
        }

        m_FrameBGRRawPtr = reinterpret_cast<uint8_t *>(av_malloc(num_bytes * sizeof(uint8_t)));
        if (!m_FrameBGRRawPtr) {
            throw std::runtime_error(std::string("Cannot allocate memory for the buffer: ") +
                                     std::to_string(num_bytes));
        }

        int ret = avpicture_fill(reinterpret_cast<AVPicture *>(m_FrameBGRPtr),
                                 m_FrameBGRRawPtr,
                                 AV_PIX_FMT_BGR24,
                                 m_CodecContextPtr->width,
                                 m_CodecContextPtr->height);
        if (ret == 0) {
            throw std::runtime_error("Failed to initialize the picture data structure.");
        }

        m_ImgConvertContextPtr = sws_getContext(m_CodecContextPtr->width,
                                                m_CodecContextPtr->height,
                                                m_CodecContextPtr->pix_fmt,
                                                m_CodecContextPtr->width,
                                                m_CodecContextPtr->height,
                                                AV_PIX_FMT_BGR24,
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
    if (m_FrameBGRPtr) {
        av_free(m_FrameBGRPtr);
    }

    if (m_FrameBGRRawPtr) {
        av_free(m_FrameBGRRawPtr);
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
Bebop::VideoStream::convertFrameToBGR()
{
    if (!m_CodecContextPtr->width || !m_CodecContextPtr->height)
        return;
    sws_scale(m_ImgConvertContextPtr,
              m_FramePtr->data,
              m_FramePtr->linesize,
              0,
              m_CodecContextPtr->height,
              m_FrameBGRPtr->data,
              m_FrameBGRPtr->linesize);
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
                    if (!reallocateBuffers()) {
                        std::cerr << "Buffer reallocation failed!" << std::endl;
                    }
                }
                convertFrameToBGR();
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

// start VideoStream class
cv::Size
Bebop::VideoStream::getOutputSize() const
{
    return cv::Size(getFrameWidth(), getFrameHeight());
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
        std::lock_guard<decltype(stream->m_FrameMutex)> guard(stream->m_FrameMutex);
        stream->m_Frame = cv::Mat(stream->getOutputSize(), CV_8UC3, (void *) stream->m_FrameBGRRawPtr);
        stream->m_NewFrame = true;
    }
    return ARCONTROLLER_OK;
}
// end VideoStream class
} // Robots
} // GeNNRobotics
