// BoB robotics includes
#include "robots/bebop/bebop.h"
#include "common/assert.h"
#include "common/logging.h"

using namespace units::angle;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;

/*
 * Little macros to make using the ARSDK's C API less verbose.
 */
#define DRONE_COMMAND(COMMAND, ...) \
    checkError(m_Device->aRDrone3->COMMAND(m_Device->aRDrone3, __VA_ARGS__))

#define DRONE_COMMAND_NO_ARG(COMMAND) \
    checkError(m_Device->aRDrone3->COMMAND(m_Device->aRDrone3))

/*
 * For when a maximum speed parameter has changed
 */
#define MAX_SPEED_CHANGED(NAME, KEY)                                         \
    {                                                                        \
        bebop->m_##NAME##Limits.onChanged(                                   \
                dict,                                                        \
                ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_##KEY##CHANGED_CURRENT, \
                ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_##KEY##CHANGED_MIN,     \
                ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_##KEY##CHANGED_MAX);    \
    }

/*
 * Throw a runtime_error if discovery device fails.
 */
inline void
checkError(eARDISCOVERY_ERROR err)
{
    if (err != ARDISCOVERY_OK) {
        throw std::runtime_error(std::string("Discovery error: ") +
                                 ARDISCOVERY_Error_ToString(err));
    }
}

namespace BoBRobotics {
namespace Robots {

/** BEGIN PUBLIC MEMBERS **/

// We also have to give declarations for these variables, because c++ is weird
constexpr degree_t Bebop::DefaultMaximumTilt;
constexpr degrees_per_second_t Bebop::DefaultMaximumYawSpeed;
constexpr meters_per_second_t Bebop::DefaultMaximumVerticalSpeed;

/*!
 * \brief Search for the drone on the network and connect to it.
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

/*!
 * \brief Land the drone and disconnect properly when the object is destroyed.
 */
Bebop::~Bebop()
{
    land();
    stopStreaming();
    disconnect();
}

/*!
 * \brief Send take-off command.
 */
void
Bebop::takeOff()
{
    LOG_INFO << "Taking off...";
    DRONE_COMMAND_NO_ARG(sendPilotingTakeOff);

    if (m_FlightEventHandler) {
        m_FlightEventHandler(true);
    }
}

/*!
 * \brief Send land command.
 */
void
Bebop::land()
{
    LOG_INFO << "Landing...";
    DRONE_COMMAND_NO_ARG(sendPilotingLanding);

    if (m_FlightEventHandler) {
        m_FlightEventHandler(false);
    }
}

/*!
 * \brief Return the an object allowing access to the drone's onboard camera.
 */
Bebop::VideoStream &
Bebop::getVideoStream()
{
    startStreaming();
    return *m_VideoStream;
}

/*!
 * \brief Return the current maximum tilt setting.
 *
 * This affects pitch and roll.
 */
degree_t
Bebop::getMaximumTilt() const
{
    return m_TiltLimits.getCurrent();
}

/*!
 * \brief Return the minimum and maximum permitted values for the maximum tilt
 *        setting.
 */
std::pair<degree_t, degree_t> &
Bebop::getTiltLimits()
{
    return m_TiltLimits.getLimits();
}

/*!
 * \brief Return the current maximum vertical speed setting.
 */
meters_per_second_t
Bebop::getMaximumVerticalSpeed() const
{
    return m_VerticalSpeedLimits.getCurrent();
}

/*!
 * \brief Return the minimum and maximum permitted values for the maximum
 *        vertical speed setting.
 */
std::pair<meters_per_second_t, meters_per_second_t> &
Bebop::getVerticalSpeedLimits()
{
    return m_VerticalSpeedLimits.getLimits();
}

/*!
 * \brief Return the current maximum yaw speed setting.
 */
degrees_per_second_t
Bebop::getMaximumYawSpeed() const
{
    return m_YawSpeedLimits.getCurrent();
}

/*!
 * \brief Return the minimum and maximum permitted values for the maximum yaw
 *        speed setting.
 */
std::pair<degrees_per_second_t, degrees_per_second_t> &
Bebop::getYawSpeedLimits()
{
    return m_YawSpeedLimits.getLimits();
}

/*!
 * \brief Set drone's pitch, for moving forwards and backwards.
 */
void
Bebop::setPitch(float pitch)
{
    BOB_ASSERT(pitch >= -1.f && pitch <= 1.f);
    DRONE_COMMAND(setPilotingPCMDPitch, round(pitch * 100.0f));
    DRONE_COMMAND(setPilotingPCMDFlag, 1);
}

/*!
 * \brief Set drone's roll, for banking left and right.
 */
void
Bebop::setRoll(float right)
{
    BOB_ASSERT(right >= -1.f && right <= 1.f);
    DRONE_COMMAND(setPilotingPCMDRoll, round(right * 100.0f));
    DRONE_COMMAND(setPilotingPCMDFlag, 1);
}

/*!
 * \brief Set drone's vertical speed for ascending/descending.
 */
void
Bebop::setVerticalSpeed(float up)
{
    BOB_ASSERT(up >= -1.f && up <= 1.f);
    DRONE_COMMAND(setPilotingPCMDGaz, round(up * 100.0f));
}

/*!
 * \brief Set drone's yaw speed.
 */
void
Bebop::setYawSpeed(float right)
{
    BOB_ASSERT(right >= -1.f && right <= 1.f);
    DRONE_COMMAND(setPilotingPCMDYaw, round(right * 100.0f));
}

//! Carry out flat trim calibration
void
Bebop::doFlatTrimCalibration()
{
    DRONE_COMMAND_NO_ARG(sendPilotingFlatTrim);
    m_FlatTrimSemaphore.wait();
}

//! Move to a new pose relative to the current one
void
Bebop::relativeMove(meter_t x, meter_t y, meter_t z, radian_t yaw)
{
    m_RelativeMoveState = Bebop::RelativeMoveState::Moving;
    DRONE_COMMAND(sendPilotingMoveBy, (float) x.value(), (float) y.value(), (float) z.value(), (float) yaw.value());
}

//! Get the status of a relative move operation
Bebop::RelativeMoveState
Bebop::getRelativeMoveState() const
{
    return m_RelativeMoveState;
}

//! Get the distance moved in the last relative move operation
std::pair<Vector3<meter_t>, radian_t>
Bebop::getRelativeMovePoseDifference() const
{
    return std::make_pair(m_RelativeMovePositionDistance, m_RelativeMoveAngleDistance);
}

//! Reset the relative move state, e.g. after reading
void
Bebop::resetRelativeMoveState()
{
    BOB_ASSERT(m_RelativeMoveState != RelativeMoveState::Moving);
    m_RelativeMoveState = RelativeMoveState::Initial;
}

/*!
 * \brief Tell the drone to take a photo and store it.
 */
void
Bebop::takePhoto()
{
    DRONE_COMMAND(sendMediaRecordPicture, 0);
}

/*!
 * \brief Add an event handler for when the drone is taking off or landing.
 */
void
Bebop::setFlightEventHandler(FlightEventHandler handler)
{
    m_FlightEventHandler = handler;
}

/** END PUBLIC MEMBERS **/
/** BEGIN PRIVATE MEMBERS **/

/*
 * Try to make connection to drone.
 */
void
Bebop::connect()
{
    // send start signal
    checkError(ARCONTROLLER_Device_Start(m_Device.get()));

    // wait for start
    Bebop::State state;
    do {
        state = getStateUpdate();
    } while (state == Bebop::State::Starting);

    if (state != Bebop::State::Running) {
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
        Bebop::State state;
        do {
            state = getStateUpdate();
        } while (state == Bebop::State::Stopping);

        LOG_WARNING_IF(state != Bebop::State::Stopped) << "Could not disconnect from drone";
    }
}

/*
 * Create the struct used by the ARSDK to interface with the drone.
 */
inline void
Bebop::createControllerDevice()
{
    // create discovery device
    static const auto deleter = [](ARDISCOVERY_Device_t *&discover) {
        ARDISCOVERY_Device_Delete(&discover);
    };
    auto derr = ARDISCOVERY_OK;
    std::unique_ptr<ARDISCOVERY_Device_t, decltype(deleter)> discover(ARDISCOVERY_Device_New(&derr), deleter);
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
                             [](ARCONTROLLER_Device_t *&dev) {
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
 * Wait for the drone to send a state-update command and return the new
 * state.
 */
inline Bebop::State
Bebop::getStateUpdate()
{
    m_StateSemaphore.wait();
    return getState();
}

/*
 * Return the drone's connection state.
 */
Bebop::State
Bebop::getState()
{
    auto err = ARCONTROLLER_OK;
    const Bebop::State state = static_cast<Bebop::State>(ARCONTROLLER_Device_GetState(m_Device.get(), &err));
    checkError(err);
    return state;
}

//! Get the drone's battery level from 0.0f to 1.0f
float
Bebop::getBatteryLevel()
{
    m_BatteryLevelSemaphore.waitOnce();
    return static_cast<float>(m_BatteryLevel) / 100.f;
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
        m_BatteryLevel = val->value.U8;
        m_BatteryLevelSemaphore.notify();
    }
}

/*
 * Empty function used to suppress default ARSDK console messages.
 */
int
Bebop::printCallback(eARSAL_PRINT_LEVEL,
                     const char*,
                     const char*,
                     va_list)
{
    // do nothing
    return 0;
}

/*
 * Invoked when the drone's connection state changes.
 */
void
Bebop::stateChanged(eARCONTROLLER_DEVICE_STATE newstate,
                    eARCONTROLLER_ERROR,
                    void *data)
{
    Bebop *bebop = reinterpret_cast<Bebop *>(data);
    bebop->m_StateSemaphore.notify(); // trigger semaphore used by getStateUpdate()

    switch (newstate) {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
        LOG_INFO << "Drone stopped";
        break;
    case ARCONTROLLER_DEVICE_STATE_STARTING:
        LOG_INFO << "Drone starting...";
        break;
    case ARCONTROLLER_DEVICE_STATE_RUNNING:
        LOG_INFO << "Drone started";
        break;
    case ARCONTROLLER_DEVICE_STATE_PAUSED:
        LOG_INFO << "Drone is paused";
        break;
    case ARCONTROLLER_DEVICE_STATE_STOPPING:
        LOG_INFO << "Drone stopping...";
        break;
    default:
        LOG_WARNING << "Unknown drone state!";
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
    case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED:
        alertStateChanged(dict);
        break;
    case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND:
        bebop->relativeMoveEnded(dict);
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
    case ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED:
        productVersionReceived(dict);
        break;
    case ARCONTROLLER_DICTIONARY_KEY_COMMON_CALIBRATIONSTATE_MAGNETOCALIBRATIONREQUIREDSTATE:
        magnetometerCalibrationStateReceived(dict);
        break;
    case ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLATTRIMCHANGED:
        bebop->m_FlatTrimSemaphore.notify();
        break;
    default:
        break;
    }
}

void
Bebop::magnetometerCalibrationStateReceived(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
{
    ARCONTROLLER_DICTIONARY_ELEMENT_t *elem = nullptr;
    HASH_FIND_STR(dict, ARCONTROLLER_DICTIONARY_SINGLE_KEY, elem);
    if (elem) {
        ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
        HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_CALIBRATIONSTATE_MAGNETOCALIBRATIONREQUIREDSTATE_REQUIRED, arg);
        if (arg && arg->value.U8) {
            LOG_WARNING << "!!! WARNING: BEBOP'S MAGNETOMETERS REQUIRE CALIBRATION !!!";
        }
    }
}

void Bebop::relativeMoveEnded(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
{
    ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *elem = nullptr;
    HASH_FIND_STR(dict, ARCONTROLLER_DICTIONARY_SINGLE_KEY, elem);
    if (!elem) {
        return;
    }

    HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DX, arg);
    if (arg) {
        m_RelativeMovePositionDistance[0] = meter_t(arg->value.Float);
    }
    HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DY, arg);
    if (arg) {
        m_RelativeMovePositionDistance[1] = meter_t(arg->value.Float);
    }
    HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DZ, arg);
    if (arg) {
        m_RelativeMovePositionDistance[2] = meter_t(arg->value.Float);
    }
    HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_DPSI, arg);
    if (arg) {
        m_RelativeMoveAngleDistance = radian_t(arg->value.Float);
    }

    HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGEVENT_MOVEBYEND_ERROR, arg);
    if (arg) {
        m_RelativeMoveState = static_cast<Bebop::RelativeMoveState>(arg->value.I32);
    }
}

void
Bebop::alertStateChanged(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
{
    ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *elem = nullptr;
    HASH_FIND_STR(dict, ARCONTROLLER_DICTIONARY_SINGLE_KEY, elem);
    if (elem) {
    HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED_STATE, arg);
    if (arg) {
        switch (arg->value.I32) {
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED_STATE_USER:
            LOG_ERROR << "Alert! User emergency alert";
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED_STATE_CUT_OUT:
            LOG_ERROR << "Alert! Drone has cut out";
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED_STATE_CRITICAL_BATTERY:
            LOG_WARNING << "Alert! Battery level is critical";
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED_STATE_LOW_BATTERY:
            LOG_WARNING << "Alert! Battery level is low";
            break;
        case ARCOMMANDS_ARDRONE3_PILOTINGSTATE_ALERTSTATECHANGED_STATE_TOO_MUCH_ANGLE:
            LOG_WARNING << "Alert! The angle of the drone is too high";
            break;
        default:
            break;
        }
    }
    }
}

void
Bebop::productVersionReceived(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
{
    ARCONTROLLER_DICTIONARY_ARG_t *arg = nullptr;
    ARCONTROLLER_DICTIONARY_ELEMENT_t *elem = nullptr;
    HASH_FIND_STR(dict, ARCONTROLLER_DICTIONARY_SINGLE_KEY, elem);
    if (elem) {
        HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED_SOFTWARE, arg);
        LOG_VERBOSE_IF(arg) << "Bebop software version: " << arg->value.String;
        HASH_FIND_STR(elem->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_SETTINGSSTATE_PRODUCTVERSIONCHANGED_HARDWARE, arg);
        LOG_VERBOSE_IF(arg) << "Bebop hardware version: " << arg->value.String;
    }
}
} // Robots
} // BoBRobotics
