// BoB robotics includes
#include "common/map_coordinate.h"

namespace BoBRobotics {
namespace MapCoordinate {
// OpenCV serialisation/deserialisation
#ifdef USE_OPENCV
void
write(cv::FileStorage &fs,
      const std::string &,
      const GPSCoordinate &gps)
{
    fs << "{"
       << "lat" << gps.lat.value()
       << "lon" << gps.lon.value()
       << "}";
}

void
read(const cv::FileNode &node,
     GPSCoordinate &gps,
     GPSCoordinate defaultValue)
{
    if (node.empty()) {
        gps = defaultValue;
    } else {
        gps.lat = units::angle::degree_t{ (double) node["lat"]};
        gps.lon = units::angle::degree_t{ (double) node["lon"]};
    }
}
#endif // USE_OPENCV
}
}