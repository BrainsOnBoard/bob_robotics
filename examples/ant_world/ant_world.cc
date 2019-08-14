// BoB robotics includes
#include "common/logging.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "hid/joystick_sfml_keyboard.h"
#include "antworld/common.h"
#include "antworld/renderer.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenGL includes
#include <GL/glew.h>

// SFML
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <cstring>
#include <memory>

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;
using namespace units::time;

// Anonymous namespace
namespace
{
void handleGLError(GLenum, GLenum, GLuint, GLenum, GLsizei, const GLchar *message,
                   const void *)
{
    throw std::runtime_error(message);
}

std::unique_ptr<HID::JoystickBase<HID::JAxis, HID::JButton>> createJoystick(sf::Window &window)
{
    try
    {
        return std::make_unique<HID::Joystick>(0.25f);
    }
    catch(std::runtime_error &ex)
    {
        LOGW << "Error opening joystick - \"" << ex.what() << "\" - using keyboard interface";
        return std::make_unique<HID::JoystickSFMLKeyboard>(window);
    }
}
}

int main(int argc, char **argv)
{
    const auto turnSpeed = 200_deg_per_s;
    const auto moveSpeed = 3_mps;
    const unsigned int width = 1024;
    const unsigned int height = 262;

    // Whether to use the 3D reconstructed Rothamsted model
    const bool useRothamstedModel = argc > 1 && strcmp(argv[1], "--rothamsted") == 0;

    // Create SFML window
    sf::Window window(sf::VideoMode{ width, height },
                      "Ant world example",
                      sf::Style::Titlebar | sf::Style::Close);

    // Enable VSync
    window.setVerticalSyncEnabled(true);
    window.setActive(true);

    // Initialize GLEW
    if(glewInit() != GLEW_OK) {
        LOGE << "Failed to initialize GLEW";
        return EXIT_FAILURE;
    }

    // Set OpenGL error callback
    glDebugMessageCallback(handleGLError, nullptr);

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.75f, 0.75f, 0.75f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_TEXTURE_2D);

    // Create renderer - increasing cubemap size to improve quality in larger window
    // and pushing back clipping plane to reduce Z fighting
    AntWorld::Renderer renderer(512, 0.1);
    if (useRothamstedModel) {
        const char *modelPath = std::getenv("ROTHAMSTED_3D_MODEL_PATH");
        if (!modelPath) {
            throw std::runtime_error("Error: ROTHAMSTED_3D_MODEL_PATH env var is not set");
        }
        renderer.getWorld().loadObj((filesystem::path(modelPath).parent_path() / "flight_1_decimate.obj").str(),
                                    0.1f,
                                    4096,
                                    GL_COMPRESSED_RGB);
    } else {
        renderer.getWorld().load(filesystem::path(argv[0]).parent_path() / "../../resources/antworld/world5000_gray.bin",
                                 { 0.0f, 1.0f, 0.0f },
                                 { 0.898f, 0.718f, 0.353f });
    }

    // Create HID device for controlling movement
    auto joystick = createJoystick(window);

    // Get world bounds and initially centre agent in world
    const auto &worldMin = renderer.getWorld().getMinBound();
    const auto &worldMax = renderer.getWorld().getMaxBound();
    meter_t x = worldMin[0] + (worldMax[0] - worldMin[0]) / 2.0;
    meter_t y = worldMin[1] + (worldMax[1] - worldMin[1]) / 2.0;
    meter_t z = worldMin[2] + (worldMax[2] - worldMin[2]) / 2.0;
    degree_t yaw = 0_deg;
    degree_t pitch = 0_deg;

    bool ant = true;
    Stopwatch moveTimer;
    moveTimer.start();
    while (window.isOpen()) {
        // Poll joystick
        joystick->update();

        // Calculate time since last iteration
        const second_t deltaTime = moveTimer.lap();

        char buffer[100];
        sprintf(buffer, "%d FPS", (int)std::round(1.0 / deltaTime.value()));
        window.setTitle(buffer);

        // Control yaw and pitch with left stick
        yaw += joystick->getState(HID::JAxis::LeftStickHorizontal) * deltaTime * turnSpeed;
        pitch += joystick->getState(HID::JAxis::LeftStickVertical) * deltaTime * turnSpeed;

        // Switch between human and ant mode using X
        if(joystick->isPressed(HID::JButton::X)) {
            ant = !ant;
        }

        // Use right trigger to control forward movement speed
        const meter_t forwardMove = moveSpeed * deltaTime * joystick->getState(HID::JAxis::RightTrigger);

        // Calculate movement delta in 3D space
        auto cosPitch = cos(pitch);
        x += forwardMove * sin(yaw) * cosPitch;
        y += forwardMove * cos(yaw) * cosPitch;
        z -= forwardMove * sin(pitch);

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render first person
        if(ant) {
            renderer.renderPanoramicView(x, y, z, yaw, pitch, 0_deg,
                                         0, 0, width, height);
        }
        else {
            renderer.renderFirstPersonView(x, y, z, yaw, pitch, 0_deg,
                                           0, 0, width, height);
        }

        // Swap front and back buffers
        window.display();
    }
    return EXIT_SUCCESS;
}
