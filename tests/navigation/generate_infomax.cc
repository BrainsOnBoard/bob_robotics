#include "generate_data.h"
#include "infomax_test.h"

using namespace BoBRobotics::Navigation;

void
generateData()
{
    InfoMaxTest algo{ TestImageSize };
    for (size_t i = 0; i < NumTestImages; i++) {
        algo.train(TestImages[i]);

        if (i < NumTrainStepsToTest) {
            const auto filePath = getTestsPath() / ("infomax_train" + std::to_string(i) + ".bin");
            writeMatrix(filePath, algo.getWeights());
        }
    }

    const InfoMax<>::VectorType netOutputs = algo.getNetOutputs(TestImages[0]);
    writeMatrix(getTestsPath() / "infomax_decision.bin", netOutputs);

    const auto &differences = algo.getImageDifferences(TestImages[0]);
    static_assert(std::is_same<float, std::remove_const_t<std::remove_reference_t<decltype(differences[0])>>>::value,
                  "Must return floats");
    writeMatrix(getTestsPath() / "infomax.bin", differences);
}

int
bobMain(int, char **)
{
    /*
     * Don't test mask case for now because of this issue:
     *      https://github.com/BrainsOnBoard/bob_robotics/issues/221
     */
    generateData();
    return EXIT_SUCCESS;
}
