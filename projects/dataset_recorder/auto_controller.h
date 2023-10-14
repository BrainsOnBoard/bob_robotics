#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/map_coordinate.h"
#include "navigation/image_database.h"
#include "robots/control/pure_pursuit_controller.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

// Standard C includes
#include <cstring>

namespace BoBRobotics {
class AutoController {
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using radian_t = units::angle::radian_t;

public:
    AutoController(const Navigation::ImageDatabase &oldDatabase,
                   millimeter_t lookahead,
                   millimeter_t wheelBaseLength,
                   millimeter_t stoppingDist)
      : m_Controller(lookahead, wheelBaseLength, stoppingDist)
      , m_Route(loadRoute(oldDatabase))
    {
        constexpr units::length::meter_t infM{ std::numeric_limits<double>::infinity() };
        auto xLims = std::make_pair(infM, -infM);
        auto yLims = xLims;

        // Add all points on route as way points
        for (const auto &utm : m_Route) {
            xLims.first = std::min(xLims.first, utm.easting);
            xLims.second = std::max(xLims.second, utm.easting);
            yLims.first = std::min(yLims.first, utm.northing);
            yLims.second = std::max(yLims.second, utm.northing);

            m_Controller.addWayPoint(utm.toVector());
        }

        LOGI << "x limits: [" << xLims.first << ", " << xLims.second << "]\n";
        LOGI << "y limits: [" << yLims.first << ", " << yLims.second << "]\n";
    }

    template<class T, class U>
    bool step(const Pose2<millimeter_t, radian_t> &pose, T &robot, U maxSpeed,
              degree_t maxTurn)
    {
        using namespace units::math;

        const auto lookPoint = m_Controller.getLookAheadPoint(pose);
        if (!lookPoint) {
            LOGE << "Robot got stuck! 😭";
            return false;
        }

        // calculate turning angle with controller
        const auto turningAngle = m_Controller.getTurningAngle(pose, lookPoint);
        if (turningAngle) {
            const auto ang = turningAngle.value();
            if (abs(ang) > maxTurn) {
                robot.move(maxSpeed, copysign(maxTurn, ang));
            } else {
                robot.move(maxSpeed, ang);
            }
        } else {
            LOGI << "Reached destination!";
            return false;
        }

        return true;
    }

    const auto &getRoute() const
    {
        return m_Route;
    }

private:
    Robots::PurePursuitController m_Controller;
    const std::vector<MapCoordinate::UTMCoordinate> m_Route;

    static std::vector<MapCoordinate::UTMCoordinate>
    loadRoute(const Navigation::ImageDatabase &database)
    {
        using namespace BoBRobotics;

        // Load route from image database
        std::vector<MapCoordinate::UTMCoordinate> route;
        for (const auto &entry : database) {
            if (std::isnan(entry.pose.x().value())) {
                continue;
            }

            route.emplace_back();
            auto &utm = route.back();
            utm.easting = entry.pose.x();
            utm.northing = entry.pose.y();
            utm.height = entry.pose.z();

            // Copy UTM zone
            memset(utm.zone, 0, 4);
            strncpy(utm.zone, entry.getExtraField("UTM zone").c_str(), 3);
        }

        // Check that we have at least two valid coordinates
        BOB_ASSERT(route.size() >= 2);

        // Sanity check: make sure all coords are in same UTM zone
        const bool allUTMZonesSame = std::all_of(route.cbegin() + 1, route.cend(), [&route](const auto &utm) {
            return strcmp(utm.zone, route[0].zone) == 0;
        });
        BOB_ASSERT(allUTMZonesSame);

        return route;
    }

}; // AutoController
} // BoBRobotics
