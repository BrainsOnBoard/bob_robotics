#include "generate_data.h"
#include "infomax_test.h"

int bobMain(int, char **)
{
    generateData<BoBRobotics::Navigation::InfoMaxTest>("infomax.bin");
    return EXIT_SUCCESS;
}
