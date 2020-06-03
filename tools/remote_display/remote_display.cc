// BoB robotics includes
#include "net/client.h"
#include "video/display.h"
#include "video/netsource.h"

using namespace BoBRobotics;

int bobMain(int, char **)
{
    Net::Client client;
    Video::NetSource video{ client };
    client.runInBackground();
    Video::Display display{ video };
    display.run();
    return EXIT_SUCCESS;
}
