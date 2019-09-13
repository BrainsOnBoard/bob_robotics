#include "state_handler.h"

// Standard C includes
#include <cmath>

// BoBRobotics includes
#include "common/logging.h"
#include "navigation/visual_navigation_base.h"

// Ardin MB includes
#include "mb_params.h"
#include "sim_params.h"

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;

//----------------------------------------------------------------------------
// StateHandler
//----------------------------------------------------------------------------
StateHandler::StateHandler(const std::string &worldFilename, const std::string &routeFilename,
                           BoBRobotics::Navigation::VisualNavigationBase &visualNavigation)
:   m_StateMachine(this, State::Invalid), m_Snapshot(SimParams::displayRenderHeight, SimParams::displayRenderWidth, CV_8UC3),
    m_Input({ SimParams::displayRenderWidth, SimParams::displayRenderHeight }, { 0, SimParams::displayRenderWidth + 10 }), m_Route(0.2f, 800),
    m_SnapshotProcessor(SimParams::displayScale, SimParams::intermediateSnapshotWidth, SimParams::intermediateSnapshotHeight, MBParams::inputWidth, MBParams::inputHeight),
    m_VectorField(20_cm), m_RandomWalkAngleDistribution(-SimParams::scanAngle.value() / 2.0, SimParams::scanAngle.value() / 2.0), m_VisualNavigation(visualNavigation)
{
    // Load world
    m_Renderer.getWorld().load(worldFilename, SimParams::worldColour, SimParams::groundColour);

    // If route is specified
    if(!routeFilename.empty()) {
        m_Route.load(routeFilename);

        // Get bounds of route
        const auto &routeMin = m_Route.getMinBound();
        const auto &routeMax = m_Route.getMaxBound();

        // Create vector field geometry to cover route bounds
        m_VectorField.createVertices(routeMin[0] - 20_cm, routeMax[0] + 20_cm, 20_cm, routeMin[1] - 20_cm, routeMax[1] + 20_cm, 20_cm);

        // Start training
        m_StateMachine.transition(State::Training);
        return;
    }

    // Move ant to initial position
    resetAntPosition();
    m_StateMachine.transition(State::FreeMovement);
}
//----------------------------------------------------------------------------
bool StateHandler::handleEvent(State state, Event event)
{
    // If this event is an update
    if(event == Event::Update) {

        // Render top down and ants eye view
        m_Renderer.renderPanoramicView(m_Pose.y(), m_Pose.x(), 0.01_m,
                                       m_Pose.yaw(), 0.0_deg, 0.0_deg,
                                       0, SimParams::displayRenderWidth + 10, SimParams::displayRenderWidth, SimParams::displayRenderHeight);
        m_Renderer.renderTopDownView(0, 0, SimParams::displayRenderWidth, SimParams::displayRenderWidth);

        // Render route
        m_Route.render(m_Pose.x(), m_Pose.y(), m_Pose.yaw());

        // Render vector field
        m_VectorField.render();

        // Read pixels from framebuffer
        m_Input.readFrame(m_Snapshot);

        // Process snapshot
        m_SnapshotProcessor.process(m_Snapshot);

        // If random walk key is pressed, transition to correct state
        if(m_KeyBits.test(KeyRandomWalk)) {
            m_StateMachine.transition(State::RandomWalk);
        }

        // If vector field key is pressed, transition to correct state
        if(m_KeyBits.test(KeyBuildVectorField)) {
            m_StateMachine.transition(State::BuildingVectorField);
        }

        cv::waitKey(1);
    }

    // If we're in training state
    if(state == State::Training) {
        if(event == Event::Enter) {
            LOGI << "Starting training";
            m_TrainPoint = 0;

            resetAntPosition();
        }
        else if(event == Event::Update) {
            // Train memory with snapshot
            m_VisualNavigation.train(m_SnapshotProcessor.getFinalSnapshot());

            // Mark results from previous training snapshot on route
            m_Route.setWaypointFamiliarity(m_TrainPoint - 1, 0.5f);//(double)numENSpikes / 20.0);

            // If GeNN isn't training and we have more route points to train
            if(m_TrainPoint < m_Route.size()) {
                // Snap ant to next snapshot point
                m_Pose = m_Route[m_TrainPoint];

                // Go onto next training point
                m_TrainPoint++;
            }
            // Otherwise, if we've reached end of route
            else {
                LOGI << "Training complete (" << m_Route.size() << " snapshots)";


                m_StateMachine.transition(State::Testing);
            }
        }
    }
    else if(state == State::Testing) {
        if(event == Event::Enter) {
            // Snap ant back to start of route, facing in starting scan direction
            resetAntPosition();
            m_Pose.yaw() -= (SimParams::scanAngle / 2.0);

            // Add initial replay point to route
            m_Route.addPoint(m_Pose.x(), m_Pose.y(), false);

            m_TestingScan = 0;

            m_MaxTestPoint = 0;
            m_NumTestErrors = 0;
            m_NumTestSteps = 0;

            m_BestTestHeading = 0.0_deg;
            m_LowestTestDifference = std::numeric_limits<float>::max();
        }
        else if(event == Event::Update) {
            // Test snapshot
            const float difference = m_VisualNavigation.test(m_SnapshotProcessor.getFinalSnapshot());

            // If this is an improvement on previous best spike count
            if(difference < m_LowestTestDifference) {
                m_BestTestHeading = m_Pose.yaw();
                m_LowestTestDifference = difference;

                //LOGI << "\tUpdated result: " << m_BestTestHeading << " is most familiar heading with " << m_LowestTestDifference << " difference";
            }

            // Go onto next scan
            m_TestingScan++;

            // If scan isn't complete
            if(m_TestingScan < SimParams::numScanSteps) {
                // Scan right
                m_Pose.yaw() += SimParams::scanStep;
            }
            else {
                LOGI << "Scan complete: " << m_BestTestHeading << " is most familiar heading with " << m_LowestTestDifference << " difference";

                // Snap ant to it's best heading
                m_Pose.yaw() = m_BestTestHeading;

                // Increment step count
                m_NumTestSteps++;

                // Move ant forward by snapshot distance
                m_Pose.x() += SimParams::snapshotDistance * units::math::sin(m_Pose.yaw());
                m_Pose.y() += SimParams::snapshotDistance * units::math::cos(m_Pose.yaw());

                // If new position means run is over - stop
                if(!checkAntPosition()) {
                    return false;
                }
                // Otherwise, reset scan
                else {
                    m_Pose.yaw() -= (SimParams::scanAngle / 2.0);
                    m_TestingScan = 0;
                    m_LowestTestDifference = std::numeric_limits<float>::max();
                }
            }
        }
    }
    else if(state == State::RandomWalk) {
        if(event == Event::Enter) {
            // Reset error and step counters
            m_NumTestErrors = 0;
            m_NumTestSteps = 0;
            m_MaxTestPoint = 0;

            resetAntPosition();
        }
        else if(event == Event::Update) {
            // Pick random heading
            m_Pose.yaw() += degree_t{ m_RandomWalkAngleDistribution(m_RNG) };

             // Increment step count
            m_NumTestSteps++;

            // Move ant forward by snapshot distance
            m_Pose.x() += SimParams::snapshotDistance * units::math::sin(m_Pose.yaw());
            m_Pose.y() += SimParams::snapshotDistance * units::math::cos(m_Pose.yaw());

            // If new position means run is over - stop
            if(!checkAntPosition()) {
                return false;
            }
        }
    }
    // Otherwise if we're in testing state
    else if(state == State::FreeMovement) {
        if(event == Event::Update) {
            // If reset key is pressed, reset ant position back to start of path
            if(m_KeyBits.test(KeyReset)) {
                resetAntPosition();
            }

            // Update heading and ant position based on keys
            if(m_KeyBits.test(KeyLeft)) {
                m_Pose.yaw() -= SimParams::antTurnStep;
            }
            if(m_KeyBits.test(KeyRight)) {
                m_Pose.yaw() += SimParams::antTurnStep;
            }
            if(m_KeyBits.test(KeyUp)) {
                m_Pose.x() += SimParams::antMoveStep * units::math::sin(m_Pose.yaw());
                m_Pose.y() += SimParams::antMoveStep * units::math::cos(m_Pose.yaw());
            }
            if(m_KeyBits.test(KeyDown)) {
                m_Pose.x() -= SimParams::antMoveStep * units::math::sin(m_Pose.yaw());
                m_Pose.y() -= SimParams::antMoveStep * units::math::cos(m_Pose.yaw());
            }
            if(m_KeyBits.test(KeyTrainSnapshot)) {
                m_VisualNavigation.train(m_Snapshot);

            }
            if(m_KeyBits.test(KeyTestSnapshot)) {
                LOGI << "Difference: " << m_VisualNavigation.test(m_Snapshot);
            }
            if(m_KeyBits.test(KeySaveSnapshot)) {
                cv::imwrite("snapshot.png", m_SnapshotProcessor.getFinalSnapshot());
            }
        }
    }
    else if(state == State::BuildingVectorField) {
        if(event == Event::Enter) {
            // Reset ant heading and move it to first vector field position
            m_CurrentVectorFieldPoint = 0;
            m_Pose.yaw() = 0_deg;
            m_TestingScan = 0;
            std::tie(m_Pose.x(), m_Pose.y()) = m_VectorField.getPoint(m_CurrentVectorFieldPoint);

            // Clear vector of novelty values
            m_VectorFieldNovelty.clear();
        }
        else if(event == Event::Update) {
            // Test snapshot
            const float difference = m_VisualNavigation.test(m_SnapshotProcessor.getFinalSnapshot());

            // Add novelty to vector
            m_VectorFieldNovelty.push_back(std::make_pair(m_Pose.yaw(), difference));

            // Go onto next scan
            m_TestingScan++;

            // If scan isn't complete
            if(m_TestingScan < SimParams::numVectorFieldSteps) {
                m_Pose.yaw() += SimParams::scanStep;
            }
            else {
                // Add novelty to vector field
                m_VectorField.setNovelty(m_CurrentVectorFieldPoint, m_VectorFieldNovelty);
                m_VectorFieldNovelty.clear();

                // Go onto next vector field point
                m_CurrentVectorFieldPoint++;

                // If there are more points to evaluate, re-enter state
                if(m_CurrentVectorFieldPoint < m_VectorField.getNumPoints()) {
                    m_Pose.yaw() = 0_deg;
                    m_TestingScan = 0;
                    std::tie(m_Pose.x(), m_Pose.y()) = m_VectorField.getPoint(m_CurrentVectorFieldPoint);
                }
                // Otherwise go back to free testing
                else {
                    m_StateMachine.transition(State::FreeMovement);
                }
            }
        }
    }
    else {
        throw std::runtime_error("Invalid state");
    }

    return true;
}
//----------------------------------------------------------------------------
bool StateHandler::checkAntPosition()
{
    // If we've reached destination
    if(m_Route.atDestination(m_Pose.x(), m_Pose.y(), SimParams::errorDistance)) {
        LOGW << "Destination reached in " << m_NumTestSteps << " steps with " << m_NumTestErrors << " errors";

        // Add final point to route
        m_Route.addPoint(m_Pose.x(), m_Pose.y(), false);

        // Stop
        return false;
    }
    // Otherwise, if we've
    else if(m_NumTestSteps >= SimParams::testStepLimit) {
        LOGW << "Failed to find destination after " << m_NumTestSteps << " steps and " << m_NumTestErrors << " errors";

        // Stop
        return false;
    }
    // Otherwise
    else {
        // Calculate distance to route
        meter_t distanceToRoute;
        size_t nearestRouteWaypoint;
        std::tie(distanceToRoute, nearestRouteWaypoint) = m_Route.getDistanceToRoute(m_Pose.x(), m_Pose.y());
        LOGI << "\tDistance to route: " << distanceToRoute * 100.0f << "cm";

        // If we are further away than error threshold
        if(distanceToRoute > SimParams::errorDistance) {
            // If we have previously reached further down route than nearest point
            // the furthest point reached is our 'best' waypoint otherwise it's the nearest waypoint
            const size_t bestWaypoint = (nearestRouteWaypoint < m_MaxTestPoint) ? m_MaxTestPoint : nearestRouteWaypoint;

            // Snap ant to the waypoint after this (clamping to size of route)
            const size_t snapWaypoint = std::min(bestWaypoint + 1, m_Route.size() - 1);
            m_Pose = m_Route[snapWaypoint];

            // Update maximum test point reached
            m_MaxTestPoint = std::max(m_MaxTestPoint, snapWaypoint);

            // Add error point to route
            m_Route.addPoint(m_Pose.x(), m_Pose.y(), true);

            // Increment error counter
            m_NumTestErrors++;
        }
        // Otherwise, update maximum test point reached and add 'correct' point to route
        else {
            m_MaxTestPoint = std::max(m_MaxTestPoint, nearestRouteWaypoint);
            m_Route.addPoint(m_Pose.x(), m_Pose.y(), false);
        }

        return true;
    }
}
//----------------------------------------------------------------------------
void StateHandler::resetAntPosition()
{
    if(m_Route.size() > 0) {
        m_Pose = m_Route[0];
    }
    else {
        m_Pose.x() = 5.0_m;
        m_Pose.y() = 5.0_m;
        m_Pose.yaw() = 270.0_deg;
    }
}
