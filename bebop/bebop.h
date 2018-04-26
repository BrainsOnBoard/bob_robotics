#pragma once

// includes for ARSDK
extern "C"
{
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
}

#include "semaphore.h"
#include <iostream>
#include <string>

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

typedef unsigned char u8;
typedef signed char i8;

namespace Parrot {
using namespace std;
using namespace Parrot;

#ifndef DUMMY_DRONE
/*
 * Wrapper around ARDISCOVERY_Device_t functionality.
 *
 * This is used for discovering a Bebop drone on the local network.
 */
class DiscoveryDevice
{
public:
    ARDISCOVERY_Device_t *dev = nullptr;

    DiscoveryDevice();
    ~DiscoveryDevice();
    void discover();

private:
    static inline void check_err(eARDISCOVERY_ERROR err)
    {
        if (err != ARDISCOVERY_OK) {
            throw std::runtime_error(std::string("Discovery error: ") +
                                     ARDISCOVERY_Error_ToString(err));
        }
    }
};
#endif // DUMMY_DRONE

/*
 * Simply throws a runtime_error with appropriate message if
 * err != ARCONTROLLER_OK.
 */
inline void
check_err(eARCONTROLLER_ERROR err)
{
    if (err != ARCONTROLLER_OK) {
        throw runtime_error(string("Controller error: ") +
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
public:
    ARCONTROLLER_Device_t *cdev = nullptr;

    Bebop();
    ~Bebop();
    void connect();
    void disconnect();
    void take_off();
    void land();
    void set_pitch(i8 pitch);
    void set_roll(i8 right);
    void set_up_down(i8 up);
    void set_yaw(i8 right);
    void take_photo();

private:
    Semaphore sem;
    bool isconnected = false;

#ifndef DUMMY_DRONE
    inline eARCONTROLLER_DEVICE_STATE get_state_update();
    inline eARCONTROLLER_DEVICE_STATE get_state();
    inline void create_cdev(DiscoveryDevice &ddev);
    inline void add_event_handlers();
    void battery_changed(ARCONTROLLER_DICTIONARY_ELEMENT_t *dict);
    static int print_callback(eARSAL_PRINT_LEVEL level,
                              const char *tag,
                              const char *format,
                              va_list va);
    static void state_changed(eARCONTROLLER_DEVICE_STATE newstate,
                              eARCONTROLLER_ERROR err,
                              void *data);
    static void cmd_received(eARCONTROLLER_DICTIONARY_KEY key,
                             ARCONTROLLER_DICTIONARY_ELEMENT_t *dict,
                             void *data);
#endif // !DUMMY_DRONE
};     // class Bebop
} // namespace Parrot
