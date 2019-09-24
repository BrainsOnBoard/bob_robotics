// BoB robotics includes
#include "net/client.h"
#include "video/display.h"
#include "video/netsource.h"

using namespace BoBRobotics;

int
main()
{
    Net::Client client;
    Video::NetSource video{ client };
    Video::Display display{ video };
    display.run();
}
