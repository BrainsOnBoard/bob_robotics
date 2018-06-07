#include "bebop.h"

namespace GeNNRobotics {
namespace Robots {
#ifndef DUMMY_DRONE
/*
 * Create object in memory.
 */
DiscoveryDevice::DiscoveryDevice()
{
    auto err = ARDISCOVERY_OK;
    dev = ARDISCOVERY_Device_New(&err);
    checkError(err);
}

/*
 * Free object memory.
 */
DiscoveryDevice::~DiscoveryDevice()
{
    ARDISCOVERY_Device_Delete(&dev);
}

/*
 * Try to discover drone.
 */
void
DiscoveryDevice::discover()
{
    checkError(ARDISCOVERY_Device_InitWifi(dev,
                                           ARDISCOVERY_PRODUCT_BEBOP_2,
                                           "bebop",
                                           BEBOP_IP_ADDRESS,
                                           BEBOP_DISCOVERY_PORT));
}
#endif // DUMMY_DRONE

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

    // try to discover drone on local network
    DiscoveryDevice ddev;
    ddev.discover();

    // object to interface with drone
    createControllerDevice(ddev);

    // to handle changes in state, incoming commands
    addEventHandlers();
#endif // DUMMY_DRONE
}

/*
 * Disconnect properly from drone when object destroyed.
 */
Bebop::~Bebop()
{
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
    checkError(ARCONTROLLER_Device_Start(m_Device));

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
        checkError(ARCONTROLLER_Device_Stop(m_Device));

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

        // free pointer
        ARCONTROLLER_Device_Delete(&m_Device);
        m_Device = nullptr;

        m_IsConnected = false;
    }
#endif
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
    if (m_IsConnected) {
        std::cout << "Taking off..." << std::endl;
#ifndef NO_FLY
        checkError(m_Device->aRDrone3->sendPilotingTakeOff(m_Device->aRDrone3));
#endif
        m_FlightEventHandler(true);
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
        m_FlightEventHandler(false);
    }
}

/*
 * Set drone's pitch, for moving forwards and backwards.
 */
void
Bebop::setPitch(int8_t pitch)
{
    if (m_IsConnected) {
        if (pitch > 100 || pitch < -100) {
            throw std::invalid_argument("Pitch must be between 100 and -100");
        }

#ifdef NO_FLY
        std::cout << "Setting pitch to " << (int) pitch << std::endl;
#else
        checkError(m_Device->aRDrone3->setPilotingPCMDPitch(m_Device->aRDrone3, pitch));
        checkError(m_Device->aRDrone3->setPilotingPCMDFlag(m_Device->aRDrone3, 1));
#endif
    }
}

/*
 * Set drone's roll, for banking left and right.
 */
void
Bebop::setRoll(int8_t right)
{
    if (m_IsConnected) {
        if (right > 100 || right < -100) {
            throw std::invalid_argument("Roll must be between 100 and -100");
        }

#ifdef NO_FLY
        std::cout << "Setting roll to " << (int) right << std::endl;
#else
        checkError(m_Device->aRDrone3->setPilotingPCMDRoll(m_Device->aRDrone3, right));
        checkError(m_Device->aRDrone3->setPilotingPCMDFlag(m_Device->aRDrone3, 1));
#endif
    }
}

/*
 * Set drone's up/down motion for ascending/descending.
 */
void
Bebop::setUpDown(int8_t up)
{
    if (m_IsConnected) {
        if (up > 100 || up < -100) {
            throw std::invalid_argument(
                    "Up/down value must be between 100 and -100");
        }

#ifdef NO_FLY
        std::cout << "Setting up/down to " << (int) up << std::endl;
#else
        checkError(m_Device->aRDrone3->setPilotingPCMDGaz(m_Device->aRDrone3, up));
#endif
    }
}

/*
 * Set drone's yaw. The drone will turn really slowly.
 */
void
Bebop::setYaw(int8_t right)
{
    if (m_IsConnected) {
        if (right > 100 || right < -100) {
            throw std::invalid_argument("Yaw must be between 100 and -100");
        }

#ifdef NO_FLY
        std::cout << "Setting yaw to " << (int) right << std::endl;
#else
        checkError(m_Device->aRDrone3->setPilotingPCMDYaw(m_Device->aRDrone3, right));
#endif
    }
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
        setUpDown(0);
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
Bebop::setFlightEventHandler(flightEventHandler handler)
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
    auto state = ARCONTROLLER_Device_GetState(m_Device, &err);
    checkError(err);
    return state;
}

/*
 * Create the struct used by the ARSDK to interface with the drone.
 */
inline void
Bebop::createControllerDevice(DiscoveryDevice &ddev)
{
    auto err = ARCONTROLLER_OK;
    m_Device = ARCONTROLLER_Device_New(ddev.dev, &err);
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
            m_Device, stateChanged, this));
    checkError(ARCONTROLLER_Device_AddCommandReceivedCallback(
            m_Device, commandReceived, this));
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

    if (key ==
        ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED) {
        Bebop *bebop = reinterpret_cast<Bebop *>(data);
        bebop->batteryChanged(dict);
    }
}
#endif // !DUMMY_DRONE
} // Robots
} // GeNNRobotics
