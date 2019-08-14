#include "state_handler.h"

// Standard C includes
#include <cmath>

// IMGUI
#include "imgui.h"

// BoBRobotics includes
#include "navigation/visual_navigation_base.h"

// Antworld includes
#include "antworld/snapshot_processor.h"

// Ardin MB includes
#include "sim_params.h"
#include "visual_navigation_ui.h"

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;

//----------------------------------------------------------------------------
// StateHandler
//----------------------------------------------------------------------------
StateHandler::StateHandler(const std::string &worldFilename, const std::string &routeFilename, float jitterSD, bool quitAfterTrain, bool autoTest,
                           BoBRobotics::AntWorld::SnapshotProcessor &snapshotProcessor, BoBRobotics::Navigation::VisualNavigationBase &visualNavigation,
                           VisualNavigationUI &visualNavigationUI)
:   m_StateMachine(this, State::Invalid), m_Snapshot(SimParams::displayRenderHeight, SimParams::displayRenderWidth, CV_8UC3),
    m_RenderTargetTopDown(SimParams::displayRenderWidth, SimParams::displayRenderWidth), m_RenderTargetPanoramic(SimParams::displayRenderWidth, SimParams::displayRenderHeight),
    m_Input(m_RenderTargetPanoramic), m_Route(0.2f, 800), m_VectorField(20_cm),
    m_PositionJitterDistributionCM(0.0f, jitterSD), m_RandomWalkAngleDistribution(-SimParams::scanAngle.value() / 2.0, SimParams::scanAngle.value() / 2.0),
    m_QuitAfterTrain(quitAfterTrain), m_AutoTest(autoTest), m_SnapshotProcessor(snapshotProcessor), m_VisualNavigation(visualNavigation), m_VisualNavigationUI(visualNavigationUI)

{
    // Load world
    if(worldFilename.substr(worldFilename.length() - 3) == "obj") {
        m_Renderer.getWorld().loadObj(worldFilename);
    }
    else {
        m_Renderer.getWorld().load(worldFilename, SimParams::worldColour, SimParams::groundColour);
    }

    // If route is specified
    if(!routeFilename.empty()) {
        loadRoute(routeFilename);

        //handleUI();
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
        // Render panoramic view to target
        m_Renderer.renderPanoramicView(m_Pose.x(), m_Pose.y(), 1.6_m,
                                       m_Pose.yaw(), m_Pose.pitch(), 0.0_deg,
                                       m_RenderTargetPanoramic);

        // Bind top-down render target
        m_RenderTargetTopDown.bind();

        // Clear render target
        m_RenderTargetTopDown.clear();

        // Render top down view to target
        // **NOTE** disable automatic binding and clearing
        m_Renderer.renderTopDownView(m_RenderTargetTopDown, false, false);

        // Render route
        m_Route.render(m_Pose.x(), m_Pose.y(), m_Pose.yaw());

        // Render vector field
        //m_VectorField.render();

        // Unbind top-down render target
        m_RenderTargetTopDown.unbind();

        // Read pixels from render target
        m_Input.readFrame(m_Snapshot);

        // Process snapshot
        m_SnapshotProcessor.process(m_Snapshot);

        // Update the final snapshot texture with the snapshot processor's final output
        m_FinalSnapshotTexture.update(m_SnapshotProcessor.getFinalSnapshot());

        // Handle UI
        if(!handleUI()) {
            return false;
        }

        // Handle visual navigation model-specific UI
        m_VisualNavigationUI.handleUI();
    }

    // If we're in training state
    if(state == State::Training) {
        if(event == Event::Enter) {
            std::cout << "Starting training" << std::endl;
            m_TrainPoint = 0;

            resetAntPosition();
        }
        else if(event == Event::Update) {
            // Open popup
            // **YUCK** OpenPopup crashes if you try and call in enter of 1st state
            if(!ImGui::IsPopupOpen("Training...")) {
                ImGui::OpenPopup("Training...");
            }

            // Train memory with snapshot
            m_VisualNavigation.train(m_SnapshotProcessor.getFinalSnapshot());

            // Handle visual navigation model-specific UI
            m_VisualNavigationUI.handleUITraining();

            // Show training progress
            if(ImGui::BeginPopupModal("Training...", nullptr, ImGuiWindowFlags_NoResize)) {
                ImGui::ProgressBar((float)m_TrainPoint / (float)m_Route.size(), ImVec2(100, 20));
                if(ImGui::Button("Stop")) {
                    m_StateMachine.transition(State::FreeMovement);
                }
                ImGui::EndPopup();
            }

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
                if(m_QuitAfterTrain) {
                    return false;
                }
                else if(m_AutoTest) {
                    m_StateMachine.transition(State::Testing);
                }
                else {
                    m_StateMachine.transition(State::FreeMovement);
                }
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

            ImGui::OpenPopup("Testing...");
        }
        else if(event == Event::Update) {
            // Test snapshot
            const float difference = m_VisualNavigation.test(m_SnapshotProcessor.getFinalSnapshot());

            // Handle visual navigation model-specific UI
            m_VisualNavigationUI.handleUITesting();

            // Show training progress
            if(ImGui::BeginPopupModal("Testing...", nullptr, ImGuiWindowFlags_NoResize)) {
                ImGui::ProgressBar(std::min(1.0f, (float)m_NumTestSteps / (float)m_Route.size()), ImVec2(100, 20));
                if(ImGui::Button("Stop")) {
                    m_StateMachine.transition(State::FreeMovement);
                }
                ImGui::EndPopup();
            }

            // If this is an improvement on previous best spike count
            if(difference < m_LowestTestDifference) {
                m_BestTestHeading = m_Pose.yaw();
                m_LowestTestDifference = difference;

                //std::cout << "\tUpdated result: " << m_BestTestHeading << " is most familiar heading with " << m_LowestTestDifference << " difference" << std::endl;
            }

            // Go onto next scan
            m_TestingScan++;

            // If scan isn't complete
            if(m_TestingScan < SimParams::numScanSteps) {
                // Scan right
                m_Pose.yaw() += SimParams::scanStep;
            }
            else {
                std::cout << "Scan complete: " << m_BestTestHeading << " is most familiar heading with " << m_LowestTestDifference << " difference" << std::endl;

                // Snap ant to it's best heading
                m_Pose.yaw() = m_BestTestHeading;

                // Increment step count
                m_NumTestSteps++;

                // Move ant forward by snapshot distance
                m_Pose.x() += SimParams::snapshotDistance * units::math::sin(m_Pose.yaw());
                m_Pose.y() += SimParams::snapshotDistance * units::math::cos(m_Pose.yaw());

                // Jitter position
                m_Pose.x() += units::length::centimeter_t(m_PositionJitterDistributionCM(m_RNG));
                m_Pose.y() += units::length::centimeter_t(m_PositionJitterDistributionCM(m_RNG));

                // If new position means run is over - stop
                if(!checkAntPosition()) {
                    if(m_AutoTest) {
                        return false;
                    }
                    else {
                        m_StateMachine.transition(State::FreeMovement);
                    }
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
    else if(state == State::BuildingRIDF) {
        if(event == Event::Enter) {
            m_Pose.yaw() -= (SimParams::scanAngle / 2.0);

            m_TestingScan = 0;

            m_RIDFNovelty.clear();
            m_RIDFNovelty.reserve(SimParams::numScanSteps);
            ImGui::OpenPopup("Calculating RIDF...");

        }
        else if(event == Event::Update) {
            // Test snapshot and add difference to vector
            m_RIDFNovelty.push_back(m_VisualNavigation.test(m_SnapshotProcessor.getFinalSnapshot()));

            // Show training progress
            if(ImGui::BeginPopupModal("Calculating RIDF...", nullptr, ImGuiWindowFlags_NoResize)) {
                ImGui::ProgressBar(std::min(1.0f, (float)m_TestingScan / (float)SimParams::numScanSteps), ImVec2(100, 20));
                if(ImGui::Button("Stop")) {
                    m_StateMachine.transition(State::FreeMovement);
                }
                ImGui::EndPopup();
            }

            // Go onto next scan
            m_TestingScan++;

            // If scan isn't complete
            if(m_TestingScan < SimParams::numScanSteps) {
                // Scan right
                m_Pose.yaw() += SimParams::scanStep;
            }
            else {
                m_StateMachine.transition(State::FreeMovement);
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
                m_StateMachine.transition(State::FreeMovement);
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
                m_Pose.pitch() -= SimParams::antPitchStep;
            }
            if(m_KeyBits.test(KeyDown)) {
                m_Pose.pitch() += SimParams::antPitchStep;
            }
            if(m_KeyBits.test(KeyForward)) {
                m_Pose.x() += SimParams::antMoveStep * units::math::sin(m_Pose.yaw());
                m_Pose.y() += SimParams::antMoveStep * units::math::cos(m_Pose.yaw());
            }
            if(m_KeyBits.test(KeyBackward)) {
                m_Pose.x() -= SimParams::antMoveStep * units::math::sin(m_Pose.yaw());
                m_Pose.y() -= SimParams::antMoveStep * units::math::cos(m_Pose.yaw());
            }
            //if(m_KeyBits.test(KeySaveSnapshot)) {
            //    cv::imwrite("snapshot.png", m_SnapshotProcessor.getFinalSnapshot());
            //}
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
void StateHandler::resetAntPosition()
{
    if(m_Route.size() > 0) {
        m_Pose = m_Route[0];
    }
    else {
        m_Pose.x() = 5.0_m;
        m_Pose.y() = 5.0_m;
        m_Pose.yaw() = 270.0_deg;
        m_Pose.pitch() = 0.0_deg;
    }
}
//----------------------------------------------------------------------------
bool StateHandler::checkAntPosition()
{
    // If we've reached destination
    if(m_Route.atDestination(m_Pose.x(), m_Pose.y(), SimParams::errorDistance)) {
        std::cerr << "Destination reached in " << m_NumTestSteps << " steps with " << m_NumTestErrors << " errors" << std::endl;

        // Add final point to route
        m_Route.addPoint(m_Pose.x(), m_Pose.y(), false);

        // Stop
        return false;
    }
    // Otherwise, if we've
    else if(m_NumTestSteps >= SimParams::testStepLimit) {
        std::cerr << "Failed to find destination after " << m_NumTestSteps << " steps and " << m_NumTestErrors << " errors" << std::endl;

        // Stop
        return false;
    }
    // Otherwise
    else {
        // Calculate distance to route
        meter_t distanceToRoute;
        size_t nearestRouteWaypoint;
        std::tie(distanceToRoute, nearestRouteWaypoint) = m_Route.getDistanceToRoute(m_Pose.x(), m_Pose.y());
        std::cout << "\tDistance to route: " << distanceToRoute * 100.0f << "cm" << std::endl;

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
void StateHandler::loadRoute(const std::string &filename)
{
    // If loading route is successful
    m_Route.load(filename);

    // Get bounds of route
    const auto &routeMin = m_Route.getMinBound();
    const auto &routeMax = m_Route.getMaxBound();

    // Create vector field geometry to cover route bounds
    //m_VectorField.createVertices(routeMin[0] - 20_cm, routeMax[0] + 20_cm, 20_cm,
    //                             routeMin[1] - 20_cm, routeMax[1] + 20_cm, 20_cm);
}
//----------------------------------------------------------------------------
bool StateHandler::handleUI()
{
    // Draw panoramic view window
    if(ImGui::Begin("Panoramic", nullptr, ImGuiWindowFlags_NoResize))
    {
        ImGui::Image((void*)m_RenderTargetPanoramic.getTexture(),
                     ImVec2(m_RenderTargetPanoramic.getWidth(), m_RenderTargetPanoramic.getHeight()),
                     ImVec2(0, 1), ImVec2(1, 0));

        if(ImGui::Button("Train")) {
            m_VisualNavigation.train(m_SnapshotProcessor.getFinalSnapshot());
        }
        ImGui::SameLine();
        if(ImGui::Button("Test")) {
            std::cout << "Difference: " << m_VisualNavigation.test(m_SnapshotProcessor.getFinalSnapshot()) << std::endl;
        }
        ImGui::SameLine();
        if(ImGui::Button("RIDF")) {
            m_StateMachine.transition(State::BuildingRIDF);
        }
    }
    ImGui::End();

    // Draw top-down view window
    if(ImGui::Begin("Top-down", nullptr, ImGuiWindowFlags_NoResize))
    {
        ImGui::Image((void*)m_RenderTargetTopDown.getTexture(),
                     ImVec2(m_RenderTargetTopDown.getWidth(), m_RenderTargetTopDown.getHeight()),
                     ImVec2(0, 1), ImVec2(1, 0));
    }
    ImGui::End();

    // Draw processed snapshot view window
    if(ImGui::Begin("Processed snapshot", nullptr, ImGuiWindowFlags_NoResize))
    {
        ImGui::Image((void*)m_FinalSnapshotTexture.getTexture(),
                     ImVec2(m_VisualNavigation.getUnwrapResolution().width * 4, m_VisualNavigation.getUnwrapResolution().height * 4));
    }
    ImGui::End();

    // Draw processed snapshot view window
    if(ImGui::Begin("RIDF", nullptr, ImGuiWindowFlags_NoResize))
    {
        ImGui::PlotLines("", m_RIDFNovelty.data(), m_RIDFNovelty.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(SimParams::numScanSteps * 5, 50));
    }
    ImGui::End();

    if(ImGui::BeginMainMenuBar()) {
        if(ImGui::BeginMenu("File")) {
            if(ImGui::BeginMenu("Open Route")) {
                char routeFilename[32];
                for(unsigned int r = 1; r <= 14; r++) {
                    sprintf(routeFilename, "ant1_route%u.bin", r);
                    if(ImGui::MenuItem(routeFilename)) {
                        const char *bobRoboticsPath = std::getenv("BOB_ROBOTICS_PATH");
                        assert(bobRoboticsPath != nullptr);
                        std::string routePath = std::string(bobRoboticsPath) + "/resources/antworld/";

                        loadRoute(routePath + routeFilename);
                    }
                }
                ImGui::EndMenu();
            }
            if(ImGui::MenuItem("Quit")) {
                return false;
            }
            ImGui::EndMenu();
        }

        if(ImGui::BeginMenu("Tools")) {
            const bool routeLoaded = (m_Route.size() > 0);
            if(ImGui::MenuItem("Train route", nullptr, false, routeLoaded)) {
                m_StateMachine.transition(State::Training);
            }

            if(ImGui::MenuItem("Test route", nullptr, false, routeLoaded)) {
                m_StateMachine.transition(State::Testing);
            }

            if(ImGui::MenuItem("Random walk", nullptr, false, routeLoaded)) {
                m_StateMachine.transition(State::RandomWalk);
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    return true;
}
