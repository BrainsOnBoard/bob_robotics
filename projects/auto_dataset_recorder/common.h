#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/map_coordinate.h"
#include "navigation/image_database.h"

// Standard C++ includes
#include <algorithm>
#include <vector>

// Standard C includes
#include <cstring>

auto
loadRoute(const BoBRobotics::Navigation::ImageDatabase &database)
{
    using namespace BoBRobotics;

    std::vector<MapCoordinate::UTMCoordinate> route;
    route.reserve(database.size());

    const auto entryToUTM = [](const Navigation::ImageDatabase::Entry &entry) {
        MapCoordinate::UTMCoordinate utm;
        utm.easting = entry.position.x();
        utm.northing = entry.position.y();
        utm.height = entry.position.z();

        // Copy UTM zone
        memset(utm.zone, 0, 4);
        strncpy(utm.zone, entry.getExtraField("UTM zone").c_str(), 3);

        return utm;
    };
    std::transform(database.begin(), database.end(), std::back_inserter(route), entryToUTM);

    // Sanity check: make sure all coords are in same UTM zone
    const bool allUTMZonesSame = std::all_of(route.cbegin() + 1, route.cend(), [&route](const auto &utm) {
        return strcmp(utm.zone, route[0].zone) == 0;
    });
    BOB_ASSERT(allUTMZonesSame);

    return route;
}
