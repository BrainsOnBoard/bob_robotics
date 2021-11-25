// BoB robotics includes
#include "navigation/image_database.h"
#include "video/randominput.h"

// Third-party includes
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::length;
using namespace units::literals;

int bobMain(int, char **)
{
    // Make a new image database with default path
    Navigation::ImageDatabase imdb;

    // Use randomly generated images
    Video::RandomInput<> cam{ { 180, 50 }, "random", /*seed=*/42 };

    /*
     * Start recording data, using a single extra field called Sensor value. You
     * can have as many or as few extra fields as you want (or none...): just
     * pass in all the names to this function.
     *
     * // No extra fields
     * auto recorder = imdb.createRouteRecorder();
     *
     * // Two extra fields
     * auto recorder = imdb.createRouteRecorder("Value 1", "Value 2");
     *
     * // ...etc.
     */
    auto recorder = imdb.createRouteRecorder("Sensor value");

    // Save camera info (not essential, but useful so we know resolution etc.)
    recorder.getMetadataWriter() << "camera" << cam;

    Vector3<millimeter_t> pos;
    pos.z() = 10_cm;
    cv::Mat image;
    double sensorValue = 0;

    // Save some random images into database at given positions
    for (int i = 0; i < 5; i++) {
        cam.readFrameSync(image);

        // Save image and info (including the reading from our extra dummy sensor)
        recorder.record(pos, 0_deg, image, sensorValue);

        pos.x() += 0.5_m;
        pos.y() += 0.25_m;
        sensorValue += 0.1;
    }

    return EXIT_SUCCESS;
}
