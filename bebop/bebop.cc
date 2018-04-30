#include "bebop.h"

namespace Parrot {
using namespace std;

#ifndef DUMMY_DRONE
/*
 * Create object in memory.
 */
DiscoveryDevice::DiscoveryDevice()
{
    auto err = ARDISCOVERY_OK;
    dev = ARDISCOVERY_Device_New(&err);
    check_err(err);
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
    check_err(ARDISCOVERY_Device_InitWifi(dev,
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
    cout << "Running in DUMMY_DRONE mode -- will not connect to drone!" << endl;
#else
#ifdef NO_FLY
    cout << "Running in NO_FLY mode -- drone will not move!" << endl;
#endif

    // silence annoying messages printed by library
    ARSAL_Print_SetCallback(print_callback);

    // try to discover drone on local network
    DiscoveryDevice ddev;
    ddev.discover();

    // object to interface with drone
    create_cdev(ddev);

    // to handle changes in state, incoming commands
    add_event_handlers();
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
#ifndef DUMMY_DRONE
    // send start signal
    check_err(ARCONTROLLER_Device_Start(cdev));

    // wait for start
    eARCONTROLLER_DEVICE_STATE state;
    do {
        state = get_state_update();
    } while (state == ARCONTROLLER_DEVICE_STATE_STARTING);

    if (state != ARCONTROLLER_DEVICE_STATE_RUNNING) {
        throw runtime_error("Could not connect to drone");
    }
#else
    cout << "Drone started" << endl;
#endif
    isconnected = true;
}

/*
 * Try to disconnect from drone.
 */
void
Bebop::disconnect()
{
#ifndef DUMMY_DRONE
    if (cdev) {
        // send stop signal
        check_err(ARCONTROLLER_Device_Stop(cdev));

        if (isconnected) {
            // wait for stop
            eARCONTROLLER_DEVICE_STATE state;
            do {
                state = get_state_update();
            } while (state == ARCONTROLLER_DEVICE_STATE_STOPPING);

            if (state != ARCONTROLLER_DEVICE_STATE_STOPPED) {
                cerr << "Warning: Could not disconnect from drone" << endl;
            }
        }

        // free pointer
        ARCONTROLLER_Device_Delete(&cdev);
        cdev = nullptr;

        isconnected = false;
    }
#endif
}

/*
 * Send take-off command.
 */
void
Bebop::take_off()
{
    if (isconnected) {
        cout << "Taking off..." << endl;
#ifndef NO_FLY
        check_err(cdev->aRDrone3->sendPilotingTakeOff(cdev->aRDrone3));
#endif
    }
}

/*
 * Send land command.
 */
void
Bebop::land()
{
    if (isconnected) {
        cout << "Landing..." << endl;
#ifndef NO_FLY
        check_err(cdev->aRDrone3->sendPilotingLanding(cdev->aRDrone3));
#endif
    }
}

/*
 * Set drone's pitch, for moving forwards and backwards.
 */
void
Bebop::set_pitch(i8 pitch)
{
    if (isconnected) {
        if (pitch > 100 || pitch < -100) {
            throw invalid_argument("Pitch must be between 100 and -100");
        }

#ifdef NO_FLY
        cout << "Setting pitch to " << (int) pitch << endl;
#else
        check_err(cdev->aRDrone3->setPilotingPCMDPitch(cdev->aRDrone3, pitch));
        check_err(cdev->aRDrone3->setPilotingPCMDFlag(cdev->aRDrone3, 1));
#endif
    }
}

/*
 * Set drone's roll, for banking left and right.
 */
void
Bebop::set_roll(i8 right)
{
    if (isconnected) {
        if (right > 100 || right < -100) {
            throw invalid_argument("Roll must be between 100 and -100");
        }

#ifdef NO_FLY
        cout << "Setting roll to " << (int) right << endl;
#else
        check_err(cdev->aRDrone3->setPilotingPCMDRoll(cdev->aRDrone3, right));
        check_err(cdev->aRDrone3->setPilotingPCMDFlag(cdev->aRDrone3, 1));
#endif
    }
}

/*
 * Set drone's up/down motion for ascending/descending.
 */
void
Bebop::set_up_down(i8 up)
{
    if (isconnected) {
        if (up > 100 || up < -100) {
            throw invalid_argument(
                    "Up/down value must be between 100 and -100");
        }

#ifdef NO_FLY
        cout << "Setting up/down to " << (int) up << endl;
#else
        check_err(cdev->aRDrone3->setPilotingPCMDGaz(cdev->aRDrone3, up));
#endif
    }
}

/*
 * Set drone's yaw. The drone will turn really slowly.
 */
void
Bebop::set_yaw(i8 right)
{
    if (isconnected) {
        if (right > 100 || right < -100) {
            throw invalid_argument("Yaw must be between 100 and -100");
        }

#ifdef NO_FLY
        cout << "Setting yaw to " << (int) right << endl;
#else
        check_err(cdev->aRDrone3->setPilotingPCMDYaw(cdev->aRDrone3, right));
#endif
    }
}

/*
 * Stops the drone from moving along all axes.
 */
void
Bebop::stop_moving()
{
    if (isconnected) {
        set_pitch(0);
        set_roll(0);
        set_yaw(0);
        set_up_down(0);
    }
}

/*
 * Tells the drone to take a photo and store it.
 */
void
Bebop::take_photo()
{
    if (isconnected) {
        cout << "Taking photo" << endl;
#ifndef DUMMY_DRONE
        check_err(cdev->aRDrone3->sendMediaRecordPicture(cdev->aRDrone3, 0));
#endif
    }
}

#ifndef DUMMY_DRONE
/*
 * Waits for the drone to send a state-update command and returns the new
 * state.
 */
inline eARCONTROLLER_DEVICE_STATE
Bebop::get_state_update()
{
    sem.wait();
    return get_state();
}

/*
 * Returns the drone's connection state.
 */
inline eARCONTROLLER_DEVICE_STATE
Bebop::get_state()
{
    auto err = ARCONTROLLER_OK;
    auto state = ARCONTROLLER_Device_GetState(cdev, &err);
    check_err(err);
    return state;
}

/*
 * Create the struct used by the ARSDK to interface with the drone.
 */
inline void
Bebop::create_cdev(DiscoveryDevice &ddev)
{
    auto err = ARCONTROLLER_OK;
    cdev = ARCONTROLLER_Device_New(ddev.dev, &err);
    check_err(err);
}

/*
 * Add callbacks for when connection state changes or a command is received
 * from the drone.
 */
inline void
Bebop::add_event_handlers()
{
    check_err(ARCONTROLLER_Device_AddStateChangedCallback(
            cdev, state_changed, this));
    check_err(ARCONTROLLER_Device_AddCommandReceivedCallback(
            cdev, cmd_received, this));
}

/*
 * Invoked by cmd_received().
 *
 * Prints the battery state whenever it changes.
 */
inline void
Bebop::battery_changed(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict)
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
        cout << "Battery: " << (int) val->value.U8 << "%" << endl;
    }
}

/*
 * Empty function used to suppress default ARSDK console messages.
 */
int
Bebop::print_callback(eARSAL_PRINT_LEVEL level,
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
Bebop::state_changed(eARCONTROLLER_DEVICE_STATE newstate,
                     eARCONTROLLER_ERROR err,
                     void *data)
{
    Bebop *bebop = reinterpret_cast<Bebop *>(data);
    bebop->sem.notify(); // trigger semaphore used by get_state_update()

    switch (newstate) {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
        cout << "Drone stopped" << endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_STARTING:
        cout << "Drone starting..." << endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_RUNNING:
        cout << "Drone started" << endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_PAUSED:
        cout << "Drone is paused" << endl;
        break;
    case ARCONTROLLER_DEVICE_STATE_STOPPING:
        cout << "Drone stopping..." << endl;
        break;
    default:
        cerr << "Unknown state!" << endl;
    }
}

/*
 * Invoked when a command is received from drone.
 */
void
Bebop::cmd_received(eARCONTROLLER_DICTIONARY_KEY key,
                    ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                    void *data)
{
    if (!dict) {
        return;
    }

    if (key ==
        ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED) {
        Bebop *bebop = reinterpret_cast<Bebop *>(data);
        bebop->battery_changed(dict);
    }
}
#endif // !DUMMY_DRONE
} // namespace Parrot
