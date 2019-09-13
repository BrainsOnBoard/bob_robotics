// BoB robotics includes
/*
This file provides a vicon based control overlay for the BetaFlight
drone.
*/

#include <array>
#include <iomanip>

#include "vicon/capture_control.h"
#include "vicon/udp.h"

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
	using degree_t = units::angle::degree_t;
	using meters_per_second = units::velocity::meters_per_second_t;

  public:

    FILE * f;

    betaflight_vicon(std::string device, int baud) :
      m_MyDrone(device, baud),
      m_Vicon(51001)
    {

        std::this_thread::sleep_for(0.1ms);

        // start streaming status data
        m_MyDrone.subscribe();

        std::this_thread::sleep_for(0.1s);

        f = fopen("/home/nvidia/log1.csv","w");

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
        const auto &velocity_crap = objectData.getVelocity();
        std::array<float,3> velocity;
        for (int i = 0; i < 3; ++i) {
          velocity[i] = float(velocity_crap[i]);
        }
        // safety!
        if (float(position[0])/1000.0 < m_RoomBounds.x[0] || float(position[0])/1000.0 > m_RoomBounds.x[1] || float(position[1])/1000.0 < m_RoomBounds.y[0] || float(position[1])/1000.0 > m_RoomBounds.y[1] || float(position[2])/1000.0 < m_RoomBounds.z[0] || float(position[2])/1000.0 > m_RoomBounds.z[1]) {
          // return to safe position (cancels model)
          this->setWaypoint(0,0,0.2,CONST_YAW);
        }

        // calc distance to m_Waypoint
        std::array<float,3> p_diff;
        for (int i = 0; i < 3; ++i) {
          p_diff[i] = m_Waypoint[i] - float(position[i])/1000.0;
        }

        std::array<float,3> targetVSetpoint;
        std::array<float,3> acceleration;

        for (int i = 0; i < 3; ++i) {
          // calculate acceleration
          acceleration[i] = m_OldVelocity[i] - velocity[i];
        }

        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << m_Waypoint[0]  << " " << m_Waypoint[1] << " " << m_Waypoint[2]  << "," << std::endl;

        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << p_diff[0] << " " << float(p_diff[1]) << "," << std::endl;


        float v_x_r = m_ModelData[7]*0.6;
        float v_y_r = -m_ModelData[4]*0.6;
        // calculate velocity setpoint for Z
        float v_z_r = m_ModelData[1]*0.4;

        float a_x_r = -m_ModelData[8]*0.056;
        float a_y_r = m_ModelData[5]*0.056;
        float a_z_r = -m_ModelData[2]*0.056;

        float pos_d_factor = 2000.0f;
        float pos_p_factor = 150.0f;

        if (m_ModelActive) {
           // override z setpoint
           //p_diff[2] = -(m_ModelData[0] - 17.0f)*0.1;
           velocity[2] = (v_z_r);
           acceleration[2] = a_z_r;
           std::cout << m_ModelData[0] << " " << v_z_r << std::endl;
           // use position from VICON and velocity control from model
           velocity[0] = (v_x_r);
           acceleration[0] = a_x_r;
           velocity[1] = (v_y_r);
           acceleration[1] = a_y_r;
           pos_d_factor = 1000.0f;
           pos_p_factor = 100.0f;
        }



        float speed = 1.0; // up to 1.5 possible
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
        }

        // CONTROL
        for (int i = 0; i < 3; ++i) {
          // calculate Integral m_IntegralTerm
          m_IntegralTerm[i] = m_IntegralTerm[i] + (m_VSetPoint[i] - float(velocity[i]));
        }

        float z_control;
        float p_thr = 30.0*(m_VSetPoint[2] - float(velocity[2]));
        float d_thr = 500.0*float(acceleration[2]);
        float i_thr = 0.2*m_IntegralTerm[2];

        float z_control_test =  (p_thr + d_thr + i_thr) / 100.0;

        if (m_VSetPoint[2]-float(velocity[2]) > 0) {
          z_control = (0.2*m_IntegralTerm[2]+30.0*(m_VSetPoint[2] - float(velocity[2]))-0.25*float(acceleration[2])) / 100.0;
        } else {
          z_control = (0.2*m_IntegralTerm[2]+30.0*(m_VSetPoint[2] - float(velocity[2]))-0.25*float(acceleration[2])) / 100.0;
        }
        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << m_VSetPoint[2] << " " << float(velocity[2]) << "," << std::endl;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << z_control << ",";
        m_MyDrone.setVerticalSpeed(z_control_test);

        float roll_control;
        float p_roll = -pos_p_factor * (-sin(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[0] - float(velocity[0])) + cos(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[1] - float(velocity[1])));
        float d_roll = -pos_d_factor * (-sin(float(attitude[0]/180.0*M_PI))*float(acceleration[0]) + cos(float(attitude[0]/180.0*M_PI))*float(acceleration[1]));
        float i_roll = -0.1 * (-sin(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[0] + cos(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[1]);
        roll_control = atan(((p_roll + d_roll + i_roll)/150.0)*1.5f)*2.0f/M_PI;
        //roll_control = (p_roll + d_roll + i_roll)/150.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << roll_control << ",";
        m_MyDrone.setRoll(roll_control);

        float pitch_control;
        float p_pitch = -pos_p_factor * (cos(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[0] - float(velocity[0])) + sin(float(attitude[0]/180.0*M_PI))*(m_VSetPoint[1] - float(velocity[1])));
        float d_pitch = -pos_d_factor * (cos(float(attitude[0]/180.0*M_PI))*float(acceleration[0]) + sin(float(attitude[0]/180.0*M_PI))*float(acceleration[1]));
        float i_pitch = -0.1 * (cos(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[0] + sin(float(attitude[0]/180.0*M_PI))*m_IntegralTerm[1]);
        pitch_control = atan(((p_pitch + d_pitch + i_pitch)/150.0)*1.5f)*2.0f/M_PI;
        //pitch_control = (p_pitch + d_pitch + i_pitch)/150.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << pitch_control << ",";
        m_MyDrone.setPitch(pitch_control);

        // circular min distance
        float route1 = m_Yawpoint-float(attitude[0]);
        float route2 = m_Yawpoint-float(attitude[0]) + 360;
        float dist = fabs(route1) < fabs(route2) ? route1 : route2;

        float yaw_control = -(dist)/90.0; // was 20.75
        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << yaw_control << ",";
        m_MyDrone.setYawSpeed(yaw_control);

        float px = float(position[0]);
        float py = float(position[1]);
        float pz = float(position[2]);
        float vx = float(velocity_crap[0]);
        float vy = float(velocity_crap[1]);
        float vz = float(velocity_crap[2]);
        float ax = float(acceleration[0]);
        float ay = float(acceleration[1]);
        float az = float(acceleration[2]);
        // logging
        fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", m_ModelData[0], v_z_r, a_z_r, z_control, pz,vz,az,0.0f,v_x_r,v_y_r,vx,vy,ax,ay,a_x_r,a_y_r, 0.0f, z_control, z_control_test,p_roll,d_roll,i_roll,m_Waypoint[0], m_Waypoint[1], m_Waypoint[2]);
        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << m_ModelData[1] << "," << m_ModelData[2] << "," << std::endl;

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
        std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << x  << " " << y << " " << z  << "," << std::endl;
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
      /*if (!deactivate_model && m_ModelActive == false)
        return;*/

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

        // start listening to the model for waypoints (deactivated on selection of a manual waypoint)
        m_ModelActive = true;
    }

    void sendModelData(double data[9], bool have) {

      if (!have) m_ModelActive = false;

        // interpret the model data to generate a waypoints
        // vertical goes between ~10-20
        //if (m_ModelActive == true) {
          //float z = 1.0-(data[0]-10.0)/10.0;
          //this->setWaypoint(m_Waypoint[0], m_Waypoint[1], z, CONST_YAW, false);
          for (int i = 0; i < 9; ++i)  this->m_ModelData[i] = data[i];
        //}
    }

    BoBRobotics::Robots::betaflight_uav m_MyDrone;
    UDPClient <ObjectDataVelocity> m_Vicon;
    RBounds m_RoomBounds;
    std::array<float,3> m_Waypoint = {{0,0,0}};
    std::array<float,3> m_NextWaypoint = {{0,0,0}};
    float m_Yawpoint = -90;
    float m_NextYawpoint = -90;
    std::array<float,3> m_VSetPoint = {{0,0,0}};
    std::array<float,3> m_OldVelocity = {{0,0,0}};
    std::array<float,3> m_IntegralTerm = {{0,0,-900}};
    bool m_ModelActive = false;
    bool m_QueuedWaypoint = false;
    double m_ModelData[9] = {0,0,0,0,0,0,0,0,0};

  };


}

}
