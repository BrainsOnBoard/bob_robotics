
#include <string>


// BoB robotics includes
#include "../../vicon/capture_control.h"
#include "../../vicon/udp.h"

#include "../../third_party/units.h"

using namespace BoBRobotics::Vicon;

class ViconDataStreamer {

private:

    UDPClient<ObjectData> *m_vicon;                          
    CaptureControl *m_viconCaptureControl;
    unsigned int m_vicon_udp_client_port;                     // udp streaming port number
    std::string m_vicon_capture_control_ip_address;           // vicon capture control ip address
    std::string m_vicon_capture_control_executable_path;      // vicon capture control program executable path
    unsigned int m_capture_control_port;  

    void setupVicon() {

        // setup vicon and capture control
        m_vicon = new UDPClient<ObjectData>(m_vicon_udp_client_port);
        m_viconCaptureControl = new CaptureControl( 
            m_vicon_capture_control_ip_address,
            m_capture_control_port,
            m_vicon_capture_control_executable_path
        );

        // keep trying to get stream of data
        while(m_vicon->getNumObjects() == 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "Waiting for object" << std::endl;
        }

        // start recording 
        if(!m_viconCaptureControl->startRecording("test1")) {
            std::cout << "failed to start recording" << std::endl;
        } else {
            std::cout << "vicon finished setting up" << std::endl;
        }
    }

public:

    ViconDataStreamer(
        unsigned int vicon_udp_client_port,
        std::string  vicon_capture_control_ip_address,
        std::string  vicon_capture_control_executable_path,
        unsigned int capture_control_port
        ):  m_vicon_udp_client_port(vicon_udp_client_port),
            m_vicon_capture_control_ip_address(vicon_capture_control_ip_address),
            m_vicon_capture_control_executable_path(vicon_capture_control_executable_path),
            m_capture_control_port(capture_control_port) {

        // setting up the vicon streaming
        setupVicon();
    }

    //! gets position of object from vicon
    BoBRobotics::Vector3<millimeter_t> getTranslation() {
        auto objectData = m_vicon->getObjectData(0);
        return objectData.getPosition();     
    }

    //! gets rotation of object from vicon
    BoBRobotics::Vector3<radian_t> getRotation() {
        auto objectData = m_vicon->getObjectData(0);
        return objectData.getAttitude();   
    }

};