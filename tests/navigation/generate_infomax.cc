#include "generate_data.h"
#include "infomax_test.h"

void
generateData(const std::string &filename)
{
    BoBRobotics::Navigation::InfoMaxTest algo{ TestImageSize };
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    const auto &differences = algo.getImageDifferences(TestImages[0]);
    static_assert(std::is_same<const float &, const decltype(differences[0]) &>::value,
                  "Must return floats");
    writeMatrix(getTestsPath() / filename, differences);
}

int
bobMain(int, char **)
{
    /*
     * Don't test mask case for now because of this issue:
     *      https://github.com/BrainsOnBoard/bob_robotics/issues/221
     */
    generateData("infomax.bin");
    return EXIT_SUCCESS;
}
