#pragma once

// BoB robotics includes
#include "antworld/camera.h"
#include "robots/simulated_tank.h"

namespace BoBRobotics {
namespace AntWorld {
using namespace units::literals;

class Tank
  : public Robots::SimulatedTank<units::length::meter_t, units::angle::degree_t> {
    using degree_t = units::angle::degree_t;
    using meter_t = units::length::meter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    /**!
     * \brief Wrapper around AntWorld::Camera so we can return a pointer to
     * 		  a Video::Input from get camera
     *
     * Eww... what a nasty hack.
     */
    class Camera
      : public Video::Input {
    public:
        Camera(Tank &tank);
        virtual bool needsUnwrapping() const override;
        virtual bool readFrame(cv::Mat &outFrame) override;
        virtual cv::Size getOutputSize() const override;
        virtual void setOutputSize(const cv::Size &) override;

    private:
        Tank &m_Tank;
    };

    Tank(const cv::Size &renderSize = { 720, 150 },
         meters_per_second_t maxVelocity = 0.3_mps,
         meter_t agentHeight = 1_cm);

    sf::Window &getWindow();
    virtual std::unique_ptr<Video::Input> getCamera() override;
    void setPose(const Pose2<meter_t, degree_t> &pose);
    const Vector3<meter_t> &getMinBound();
    const Vector3<meter_t> &getMaxBound();

private:
    std::unique_ptr<sf::Window> m_Window;
    Renderer m_Renderer;
    AntWorld::Camera m_Camera;
    const meter_t m_AgentHeight;

protected:
    virtual void updatePose() override;
}; // Tank
} // AntWorld
} // BoBRobotics
