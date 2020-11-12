#include "common.h"
#include "navigation/generate_images.h"
#include "navigation/PerfectMemoryRotater__.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"

TEST(PerfectMemory, AllDifferences)
{
    generateImages();

    BoBRobotics::Navigation::PerfectMemoryRotater<> pm{ TestImageSize };
    for (const auto &image : TestImages) {
        pm.train(image);
    }

    const auto &differences = pm.getImageDifferences(TestImages[0]);
    for (int snap = 0; snap < (int) NumTestImages; snap++) {
        for (int col = 0; col < TestImageSize.width; col++) {
            EXPECT_FLOAT_EQ(differences(snap, col), trueDifferences[snap][col]);
        }
    }
}
