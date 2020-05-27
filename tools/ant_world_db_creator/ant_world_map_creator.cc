// BoB robotics includes
#include "plog/Log.h"
#include "common/path.h"
#include "antworld/common.h"
#include "antworld/renderer.h"
#include "antworld/route_continuous.h"
#include "video/opengl/opengl.h"

// Third-party includes
#include "third_party/path.h"

// OpenGL includes
#include <GL/glew.h>

// SFML includes
#include <SFML/Graphics.hpp>

using namespace BoBRobotics;

namespace {
void handleGLError(GLenum, GLenum, GLuint, GLenum severity, GLsizei, const GLchar *message, const void *)
{
    if (severity == GL_DEBUG_SEVERITY_HIGH) {
        LOGE << message;
    }
    else if (severity == GL_DEBUG_SEVERITY_MEDIUM) {
        LOGW << message;
    }
    else if (severity == GL_DEBUG_SEVERITY_LOW) {
        LOGI << message;
    }
    else {
        LOGD << message;
    }
}
}


int bobMain(int argc, char **argv)
{
    const unsigned int renderWidth = 1050;
    const unsigned int renderHeight = 1050;
    const bool oldAntWorld = argc > 1 && strcmp(argv[1], "--old-ant-world") == 0;

    // Create SFML window
    sf::Window window(sf::VideoMode(renderWidth, renderHeight),
                      "Ant world",
                      sf::Style::Titlebar | sf::Style::Close);

    // Enable VSync
    window.setVerticalSyncEnabled(true);
    window.setActive(true);

    // Initialize GLEW
    if(glewInit() != GLEW_OK) {
        LOGE << "Failed to initialize GLEW";
        return EXIT_FAILURE;
    }

    glDebugMessageCallback(handleGLError, nullptr);

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_TEXTURE_2D);

    // **NOTE** because 1050 isn't a multiple of 4 - need to tweak pack alignement
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // Create renderer
    AntWorld::Renderer renderer;
    const auto antWorldPath = Path::getResourcesPath() / "antworld";
    if (oldAntWorld) {
        renderer.getWorld().load(antWorldPath / "world5000_gray.bin",
                                 { 0.0f, 1.0f, 0.0f },
                                 { 0.898f, 0.718f, 0.353f });
    } else {
        renderer.getWorld().loadObj(antWorldPath / "seville_vegetation_downsampled.obj");
    }

    // Create input to read snapshots from screen
    Video::OpenGL input({ renderWidth, renderHeight });

    // Host OpenCV array to hold pixels read from screen
    cv::Mat map(renderHeight, renderWidth, CV_8UC3);


    // Clear colour and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render first person
    renderer.renderTopDownView(0, 0, renderWidth, renderHeight);

    // Read snapshot
    input.readFrame(map);

    cv::imwrite("world5000.png", map);

    return EXIT_SUCCESS;
}
