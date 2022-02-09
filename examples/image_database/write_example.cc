// BoB robotics includes
#include "navigation/image_database.h"
#include "video/randominput.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

using namespace BoBRobotics;
using namespace units::length;
using namespace units::literals;

int bobMain(int argc, char **argv)
{
    // Make a new image database with default path
    Navigation::ImageDatabase imdb;

    // Use randomly generated images
    Video::RandomInput<> cam{ { 640, 480 }, "random", /*seed=*/42 };

    /*
     * Start recording data, using a single extra field called Sensor value. You
     * can have as many or as few extra fields as you want (or none...): just
     * pass in all the names to this function.
     *
     * // No extra fields
     * auto recorder = imdb.createRouteRecorder("jpg");
     *
     * // Two extra fields
     * auto recorder = imdb.createRouteRecorder("jpg", { "Value 1", "Value 2" });
     *
     * // ...etc.
     *
     * You can also record databases as video files by using the syntax below.
     */
    std::unique_ptr<Navigation::ImageDatabase::RouteRecorder> recorder;
    if (argc == 2 && strcmp(argv[1], "video") == 0) {
        LOGI << "Saving video file";
        recorder = imdb.createVideoRouteRecorder(cam.getOutputSize(), 5_Hz, "avi", "XVID", { "Sensor value" });
    } else {
        recorder = imdb.createRouteRecorder("png", { "Sensor value" });
    }

    // Save camera info (not essential, but useful so we know resolution etc.)
    recorder->getMetadataWriter() << "camera" << cam;

    Vector3<millimeter_t> pos;
    pos.z() = 10_cm;
    cv::Mat image;
    double sensorValue = 0;

    // Save some random images into database at given positions
    for (int i = 0; i < 5; i++) {
        cam.readFrameSync(image);

        // Save image and info (including the reading from our extra dummy sensor)
        recorder->record(pos, image, sensorValue);

        pos.x() += 0.5_m;
        pos.y() += 0.25_m;
        sensorValue += 0.1;
    }

    return EXIT_SUCCESS;
}
