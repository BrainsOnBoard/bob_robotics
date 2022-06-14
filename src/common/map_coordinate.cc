// BoB robotics includes
#include "common/map_coordinate.h"

namespace BoBRobotics {
namespace MapCoordinate {
const Ellipsoid WGS84::ellipsoid{units::length::meter_t(6378137), units::length::meter_t(6356752.314245)};

const Transform OSGB36::transformFromWGS84{units::length::meter_t(-446.448), units::length::meter_t(125.157), units::length::meter_t(-542.060),
                                           20.4894, units::angle::arcsecond_t(-0.1502), units::angle::arcsecond_t(-0.2470), units::angle::arcsecond_t(-0.8421)};
const Ellipsoid OSGB36::ellipsoid{units::length::meter_t(6377563.396), units::length::meter_t(6356256.909)};
} // MapCoordinate
} // BoBRobotics
