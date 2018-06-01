#pragma once

// C++ includes
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNNRobotics includes
#include "../imgproc/opencv_unwrap_360.h"
#include "../net/node.h"
#include "../net/socket.h"
#include "../third_party/path.h"

namespace GeNNRobotics {
namespace Video {
#define DefaultCameraName "unknown_camera"

class Input
{
public:
    virtual ~Input()
    {}

    template<typename... Ts>
    ImgProc::OpenCVUnwrap360 createDefaultUnwrapper(Ts &&... args)
    {
        cv::Size unwrapRes(std::forward<Ts>(args)...);

        // Create unwrapper
        ImgProc::OpenCVUnwrap360 unwrapper(getOutputSize(), unwrapRes);

        const std::string name = getCameraName();
        const std::string fileName = name + ".yaml";
        filesystem::path filePath(fileName);

        // first check if file exists in working directory
        if (!filePath.exists()) {
            // next check if there is a local GeNN_Robotics folder (i.e. git
            // submodule)
            const filesystem::path paramsDir = filesystem::path("imgproc") / "unwrapparams";

            filePath = filesystem::path("GeNN_Robotics") / paramsDir / fileName;
            if (!filePath.exists()) {
                // lastly look for environment variable pointing to
                // GeNN_Robotics
                static const char *envVarName = "GENN_ROBOTICS_PATH";
                const char *env = std::getenv(envVarName);
                if (!env) {
                    throw std::runtime_error(std::string(envVarName) +
                                             " environment variable is not set and unwrap "
                                             "parameters file could not be found locally");
                }

                filePath = filesystem::path(env) / paramsDir / fileName;
                if (!filePath.exists()) {
                    throw std::runtime_error(
                            "Could not find unwrap parameters file for " +
                            name);
                }
            }
        }

        // read unwrap parameters from file
        std::cout << "Loading unwrap parameters from " << filePath.str() << std::endl;
        cv::FileStorage fs(filePath.str(), cv::FileStorage::READ);
        unwrapper << fs;
        fs.release();
        return unwrapper;
    }

    virtual const std::string getCameraName() const
    {
        return DefaultCameraName;
    }

    virtual cv::Size getOutputSize() const
    {
        throw std::runtime_error("This camera does not support getting its resolution");
    }

    const virtual bool needsUnwrapping()
    {
        // only panoramic cameras are defined with the camera name specified
        return getCameraName() != DefaultCameraName;
    }

    virtual bool readFrame(cv::Mat &outFrame) = 0;

    virtual void setOutputSize(const cv::Size &size)
    {
        throw std::runtime_error("This camera's resolution cannot be changed at runtime");
    }

    void streamToNetwork(Net::Node &node)
    {
        // handle incoming IMG commands
        node.addCommandHandler("IMG", [this] (Net::Node &node, const Net::Command& command) {
            onCommandReceived(node, command);
        });
    }

protected:
    /*
     * Default constructor is protected to stop users directly creating objects
     * of this class.
     */
    Input()
    {}

private:
    std::thread m_ImageThread;

    void onCommandReceived(Net::Node &node, const Net::Command &command)
    {
        if (command[1] != "START") {
            throw Net::bad_command_error();
        }

        // ACK the command and tell client the camera resolution
        cv::Size res = getOutputSize();
        node.getSocket()->send("IMG PARAMS " + std::to_string(res.width) + " " +
                               std::to_string(res.height) + " " +
                               getCameraName() + "\n");

        // start thread to transmit images in background
        m_ImageThread = std::thread([this](Net::Node *n) { runImageSink(n); }, &node);
    }

    void runImageSink(Net::Node *node)
    {
        cv::Mat frame;
        std::vector<uchar> buffer;
        Net::Socket *sock = node->getSocket();
        while (readFrame(frame)) {
            cv::imencode(".jpg", frame, buffer);
            sock->send("IMG FRAME " + std::to_string(buffer.size()) + "\n");
            sock->send(buffer.data(), buffer.size());
        }
    }
}; // Input
} // Video
} // GeNNRobotics
