// BoB robotics includes
#include "navigation/image_database.h"

using namespace BoBRobotics;

int
main(int argc, char **argv)
{
    BOB_ASSERT(argc == 2);
    Navigation::ImageDatabase database(argv[1]);
    database.unwrap("unwrapped_" + database.getName(), { 720, 58 });
}