/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <fstream>

using namespace gazebo;
/// \brief Rotor class
class Rotor
{
    /// \brief Constructor
public:
    Rotor()
    {
        m_Pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
    }

    /// \brief rotor id
    int m_Id = 0;

    /// \brief Max rotor propeller RPM.
    double m_MaxRpm = 838.0;

    /// \brief Next command to be applied to the propeller
    double m_Cmd = 0;

    /// \brief Velocity PID for motor control
    common::PID m_Pid;

    /// \brief Control propeller joint.
    std::string m_JointName;

    /// \brief Control propeller joint.
    physics::JointPtr m_Joint;

    /// \brief direction multiplier for this rotor
    double m_Multiplier = 1;

    /// \brief unused coefficients
    double m_RotorVelocitySlowdownSim;
};

namespace gazebo {

class GAZEBO_VISIBLE GazeboQuadCopterPlugin : public ModelPlugin
{
public:

    // Documentation Inherited.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr m_UpdateConnection;

    /// \brief Pointer to the model;
    physics::ModelPtr m_Model;

    /// \brief array of propellers
    std::vector<Rotor> m_Rotors;

    /// \brief keep track of controller update sim-time.
    gazebo::common::Time m_LastControllerUpdateTime;

    /// \brief Controller update mutex.
    std::mutex m_Mutex;

    /// \brief Pointer to an IMU sensor
    sensors::ImuSensorPtr m_ImuSensor;

    /// \brief Flight control commands received
    double m_Thrust, m_Roll, m_Pitch, m_Yaw;

private:
    /// \brief Receive flight control commands
    void OnMsg(ConstQuaternionPtr &_msg);

    /// \brief  Calculate and apply forces at every tick
    void OnUpdate();
    
    /// \brief Calculate motor actuation values
    /// \param[in] _dt time step size since last update.
    void MotorMixing(const double _dt);

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    void ApplyMotorForces(const double _dt);

    /// \brief Reset PID Joint controllers.
    void ResetPIDs();

    /// \brief Receive motor commands from GazeboQuadCopter
    void ReceiveMotorCommand();

    /// \brief A node used for transport
    transport::NodePtr m_Node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr m_Sub;

    /// \brief PID controllers for thrust, roll, pitch and yaw
    common::PID m_ThrustPID, m_RollPID, m_PitchPID, m_YawPID;

    /// \brief Flight log file
    std::ofstream m_Logfile;

    /// \brief PID target values
    ignition::math::v4::Pose3d m_LoiterReference;
};
}