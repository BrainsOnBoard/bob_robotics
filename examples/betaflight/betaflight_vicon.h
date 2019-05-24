// BoB robotics includes
/*
This file provides a vicon based control overlay for the BetaFlight
drone.
*/

#include <iomanip>

#include "../../vicon/capture_control.h"
#include "../../vicon/udp.h"

#define CONST_YAW -1001

namespace BoBRobotics {
namespace Robots
{

  // limits of the room - default to a (hopefully) safe 1m^3
  struct RBounds {
    float x[2] = {-0.5,0.5};
    float y[2] = {-0.5,0.5};
    float z[2] = {0,1};
  };

  using namespace BoBRobotics::Vicon;

  class betaflight_vicon {

  public:

    betaflight_vicon(std::string device, int baud) :
      m_MyDrone(device, baud),
      m_Vicon(51001)
    {

        std::this_thread::sleep_for(0.1ms);

        // start streaming status data
        m_MyDrone.subscribe();

        std::this_thread::sleep_for(0.1s);

    }

    void armDrone() {m_MyDrone.armDrone();}
    void disarmDrone() {m_MyDrone.disarmDrone();}

    void printStatus() {

      if (m_MyDrone.getArmStateAsString().size() > 0) {
          std::cout << m_MyDrone.getArmStateAsString() << std::endl;
      }
      std::cout << "[V:" << std::setw(4) << m_MyDrone.getVoltage() << " ";

      if (m_Vicon.getNumObjects() == 1) {

        auto objectData = m_Vicon.getObjectData(0);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();

        for (int i = 0; i < 3; ++i) {
          std::cout << std::setw(5) << std::fixed << std::setprecision(2) << float(position[i])/1000.0 << ", ";
        }
        for (int i = 0; i < 3; ++i) {
          std::cout << std::setw(6) << std::fixed << std::setprecision(2) << float(attitude[i]);
          if (i < 2) std::cout << ", ";
        }

      } else {
        std::cout << "NO VICON DATA";
      }
      std::cout << "]" << std::endl;

    }

    void sendCommands(bool controlOn) {

      if (controlOn) {
        // update control
        auto objectData = m_Vicon.getObjectData(0);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();
        const auto &velocity = objectData.getVelocity();

        // calc distance to m_Waypoint
        Vector3 <float> p_diff;
        for (int i = 0; i < 3; ++i) {
          p_diff[i] = m_Waypoint[i] - float(position[i])/1000.0;
        }

        Vector3 < float > targetVSetpoint;
        Vector3 < meters_per_second_t > acceleration;

        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << p_diff[0] << " " << float(p_diff[1]) << "," << std::endl;

        // calculate velocity setpoint for Z
        float speed = 1.0;
        for (int i = 2; i < 3; ++i) {
          if (p_diff[i] > 0.4) {
            targetVSetpoint[i] = 0.5*speed;
          } else if (p_diff[i] < -0.4) {
            targetVSetpoint[i] = -0.5*speed;
          } else {
            targetVSetpoint[i] = p_diff[i] * 1.25 *speed;
          }
        }

        // calculate velocity setpoints for X/Y
        // calculate p_diff_mag
        float p_diff_mag = sqrt(pow(p_diff[0],2) + pow(p_diff[1],2));
        if (p_diff_mag > 0.4) {
          // resolve into X and Y
          targetVSetpoint[0] = 0.5*speed*p_diff[0]/p_diff_mag;
          targetVSetpoint[1] = 0.5*speed*p_diff[1]/p_diff_mag;
        } else {
          targetVSetpoint[0] = 1.25*speed*p_diff[0];
          targetVSetpoint[1] = 1.25*speed*p_diff[1];
          //targetVSetpoint[i] = p_diff[i] * 1.25 *speed;
        }

        if (fabs(p_diff[2]) < 0.1 && p_diff_mag < 0.1 && m_QueuedWaypoint) {
            m_QueuedWaypoint = false;
            m_Waypoint[0] = m_NextWaypoint[0];
            m_Waypoint[1] = m_NextWaypoint[1];
            m_Waypoint[2] = m_NextWaypoint[2];
            // bounds check yaw
            if (!(m_NextYawpoint < -180.0 || m_NextYawpoint > 180.0)) {
              m_Yawpoint = m_NextYawpoint;
            }
        }

        // for all
        for (int i = 0; i < 3; ++i) {
          // smooth the v setpoint to avoid jerks
          m_VSetPoint[i] = m_VSetPoint[i]*0.9 + targetVSetpoint[i]*0.1;
          // calculate acceleration
          acceleration[i] = m_OldVelocity[i] - velocity[i];
        }

        // CONTROL
        for (int i = 0; i < 3; ++i) {
          // calculate Integral m_IntegralTerm
          m_IntegralTerm[i] = m_IntegralTerm[i] + (m_VSetPoint[i] - float(velocity[i]));
        }

        float z_control;
        if (m_VSetPoint[2]-float(velocity[2]) > 0) {
          z_control = (0.2*m_IntegralTerm[2]+45.0*(m_VSetPoint[2] - float(velocity[2]))+0.25*float(acceleration[2])) / 100.0;
        } else {
          z_control = (0.2*m_IntegralTerm[2]+45.0*(m_VSetPoint[2] - float(velocity[2]))-0.25*float(acceleration[2])) / 100.0;
        }
        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << m_VSetPoint[2] << " " << float(velocity[2]) << "," << std::endl;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << z_control << ",";
        m_MyDrone.setVerticalSpeed(z_control);

        float roll_control;
        float p_roll = -150.0 * (-sin(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[0] - float(velocity[0])) + cos(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[1] - float(velocity[1])));
        float d_roll = 20.0 * (-sin(float(attitude[0]/180.0*M_PI))*float(acceleration[0]) + cos(float(attitude[0]/180.0*M_PI))*float(acceleration[1]));
        float i_roll = -0.1 * (-sin(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[0] + cos(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[1]);
        roll_control = (p_roll + d_roll + i_roll)/150.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << roll_control << ",";
        m_MyDrone.setRoll(roll_control);

        float pitch_control;
        float p_pitch = -150.0 * (cos(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[0] - float(velocity[0])) + sin(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[1] - float(velocity[1])));
        float d_pitch = 20.0 * (cos(float(attitude[0]/180.0*M_PI))*float(acceleration[0]) + sin(float(attitude[0]/180.0*M_PI))*float(acceleration[1]));
        float i_pitch = -0.1 * (cos(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[0] + sin(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[1]);
        pitch_control = (p_pitch + d_pitch + i_pitch)/150.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << pitch_control << ",";
        m_MyDrone.setPitch(pitch_control);

        // circular min distance
        float route1 = m_Yawpoint-float(attitude[0]);
        float route2 = m_Yawpoint-float(attitude[0]) + 360;
        float dist = fabs(route1) < fabs(route2) ? route1 : route2;

        float yaw_control = -(dist)/90.0; // was 20.75
        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << yaw_control << ",";
        m_MyDrone.setYawSpeed(yaw_control);

        //std::cout << std::endl;

        m_OldVelocity = velocity;
      }

      // NOTE: this is not currently a safe way of detecting dropout of VICON!
      if (m_Vicon.getNumObjects() == 1) {
        m_MyDrone.sendCommands();
      }
      // wait so we do not overload the drone
      std::this_thread::sleep_for(10ms);
    }

    void setRoomBounds(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) {

        m_RoomBounds.x[0] = x_min;
        m_RoomBounds.x[1] = x_max;
        m_RoomBounds.y[0] = y_min;
        m_RoomBounds.y[1] = y_max;
        m_RoomBounds.z[0] = z_min;
        m_RoomBounds.z[1] = z_max;

    }

    void setWaypoint(float x, float y, float z, float yaw, bool deactivate_model = true) {

      // unless we get a model input - turn off model control
      if (deactivate_model) {
        m_ModelActive = false;
      }
      // don't listen to the model unless model is active
      if (!deactivate_model && m_ModelActive == false)
        return;

      if (x < m_RoomBounds.x[0] || x > m_RoomBounds.x[1] || y < m_RoomBounds.y[0] || y > m_RoomBounds.y[1] || z < m_RoomBounds.z[0] || z > m_RoomBounds.z[1]) {
        std::cout << "Attempted to move outside of room bounds" << std::endl;
        return;
      } else {
          m_Waypoint[0] = x;
          m_Waypoint[1] = y;
          m_Waypoint[2] = z;
          // bounds check yaw
          if (!(yaw < -180.0 || yaw > 180.0)) {
            m_Yawpoint = yaw;
          }
      }

    }

    void setNextWaypoint(float x, float y, float z, float yaw, bool deactivate_model = true) {

      // unless we get a model input - turn off model control
      if (deactivate_model) {
        m_ModelActive = false;
      }
      // don't listen to the model unless model is active
      if (!deactivate_model && m_ModelActive == false)
        return;

      // q up next waypoint
      if (x < m_RoomBounds.x[0] || x > m_RoomBounds.x[1] || y < m_RoomBounds.y[0] || y > m_RoomBounds.y[1] || z < m_RoomBounds.z[0] || z > m_RoomBounds.z[1]) {
        std::cout << "Attempted to move outside of room bounds" << std::endl;
        return;
      } else {
          m_QueuedWaypoint = true;

          m_NextWaypoint[0] = x;
          m_NextWaypoint[1] = y;
          m_NextWaypoint[2] = z;
          // bounds check yaw
          //if (!(yaw < -180.0 || yaw > 180.0)) {
            m_NextYawpoint = yaw;
          //}
      }

    }

    bool hasQueuedWaypoint() {
      return m_QueuedWaypoint;
    }

    void activateModel() {

        // start listening to teh model for waypoints (deactivated on selection of a manual waypoint)

    }

    BoBRobotics::Robots::betaflight_uav m_MyDrone;
    UDPClient <ObjectDataVelocity> m_Vicon;
    RBounds m_RoomBounds;
    Vector3 < float > m_Waypoint = {0,0,0};
    Vector3 < float > m_NextWaypoint = {0,0,0};
    float m_Yawpoint = -90;
    float m_NextYawpoint = -90;
    Vector3 < float > m_VSetPoint = {0,0,0};
    Vector3 <meters_per_second_t> m_OldVelocity = {0_mps,0_mps,0_mps};
    Vector3  < float > m_IntegralTerm = {0,0,0};
    bool m_ModelActive = false;
    bool m_QueuedWaypoint = false;

  };


}

}
