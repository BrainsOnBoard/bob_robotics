#include "generate_data.h"
#include "infomax_test.h"

int bobMain(int, char **)
{
    /*
     * Don't test mask case for now because of this issue:
     *      https://github.com/BrainsOnBoard/bob_robotics/issues/221
     */
    generateDataRaw<BoBRobotics::Navigation::InfoMaxTest>("infomax.bin", {});
    return EXIT_SUCCESS;
}
