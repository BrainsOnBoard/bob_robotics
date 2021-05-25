#include "common.h"
#include "navigation/generate_images.h"
#include "navigation/infomax_test.h"
#include "test_algo.h"

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

const auto InitialWeights = InfoMax<>::generateInitialWeights(TestImageSize.width * TestImageSize.height, 100, /*seed=*/42);

void testSampleImage(const std::string &filename, const ImgProc::Mask &mask)
{
    constexpr float Precision = 1e-4f;

    const auto filepath = Path::getProgramDirectory() / "navigation" / filename;
    const auto outputLayer = readMatrix<float>(filepath);

    InfoMaxTest algo{ TestImageSize };
    algo.setMask(mask);
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    const auto &differences = algo.getImageDifferences(TestImages[0]);
    static_assert(std::is_same<const float &, const decltype(differences[0]) &>::value,
                  "Differences must be floats");
    ASSERT_EQ(differences.size(), outputLayer.rows());

    SCOPED_TRACE("Comparing differences");
    for (int i = 0; i < differences.size(); i++) {
        EXPECT_NEAR(differences[i], outputLayer(i), Precision);
    }
}

TEST(InfoMax, SampleImage)
{
    testSampleImage("infomax.bin", {});
}

TEST(InfoMax, SampleImageMask)
{
    testSampleImage("mask_infomax.bin", TestMask);
}

TEST(InfoMax, ZerosInDecisionMask)
{
    InfoMaxTest algo{ TestImageSize };
    algo.setMask(TestMask);
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    SCOPED_TRACE("Checking for zeros");
    const Eigen::VectorXf dec = algo.getDecision(TestImages[0], TestMask);
    // std::cout << dec << "\n";
    auto maskIt = TestMask.get().begin<uint8_t>();
    auto decIt = dec.data();
    for (int i = 0; i < TestImageSize.width * TestImageSize.height; i++) {
        if (!maskIt[i]) {
            EXPECT_FLOAT_EQ(decIt[i], 0.f);
        }
    }
}

// Check that the columns have means of approx 0 and SDs of approx 1
TEST(InfoMax, RandomWeightsDistribution)
{
    constexpr float Precision = 1e-6f;

    const auto means = InitialWeights.colwise().mean();
    for (int i = 0; i < means.size(); i++) {
        EXPECT_NEAR(means[i], 0.f, Precision);
    }

    const auto stds = (InitialWeights.array() * InitialWeights.array()).colwise().mean().sqrt();
    for (int i = 0; i < stds.size(); i++) {
        EXPECT_NEAR(stds[i], 1.f, Precision);
    }
}

// Check that too high a learning rate causes weights to blow up
TEST(InfoMax, ExplodingWeights)
{
    InfoMaxRotater<> infomax{ TestImageSize, InitialWeights, /*learningRate=*/0.1f };

    EXPECT_THROW({
        for (const auto &image : TestImages) {
            infomax.train(image);
        }
    }, WeightsBlewUpError);
}

// Check that using a sensible learning rate doesn't throw an exception
TEST(InfoMax, NonExplodingWeights)
{
    InfoMaxRotater<> infomax{ TestImageSize, InitialWeights, /*learningRate=*/1e-5f };

    EXPECT_NO_THROW({
        for (const auto &image : TestImages) {
            infomax.train(image);
        }
    });
}
