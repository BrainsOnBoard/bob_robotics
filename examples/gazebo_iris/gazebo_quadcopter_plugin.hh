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
#ifndef GAZEBO_PLUGINS_GazeboQuadCopterPlugin_HH_
#define GAZEBO_PLUGINS_GazeboQuadCopterPlugin_HH_

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
        // most of these coefficients are not used yet.
        this->m_RotorVelocitySlowdownSim = this->m_KDefaultRotorVelocitySlowdownSim;
        this->m_FrequencyCutoff = this->m_KDefaultFrequencyCutoff;
        this->m_SamplingRate = this->m_KDefaultSamplingRate;

        this->m_Pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
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

    double m_FrequencyCutoff;

    double m_SamplingRate;

    ignition::math::OnePole<double> m_VelocityFilter;

    static double m_KDefaultRotorVelocitySlowdownSim;

    static double m_KDefaultFrequencyCutoff;

    static double m_KDefaultSamplingRate;
};

namespace gazebo {

class GAZEBO_VISIBLE GazeboQuadCopterPlugin : public ModelPlugin
{
    /// \brief Constructor.
public:
    GazeboQuadCopterPlugin();

    /// \brief Destructor.
    ~GazeboQuadCopterPlugin();

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

    double m_Thrust; double m_Roll; double m_Pitch; double m_Yaw;

private:
    void OnMsg(ConstQuaternionPtr &_msg);

    void OnUpdate();
    
    void MotorMixing(const double _dt);

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    void ApplyMotorForces(const double _dt);

    /// \brief Reset PID Joint controllers.
    void ResetPIDs();

    /// \brief Receive motor commands from GazeboQuadCopter
    void ReceiveMotorCommand();

    /// \brief Send state to GazeboQuadCopter
    void SendState() const;

    /// \brief A node used for transport
    transport::NodePtr m_Node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr m_Sub;


    common::PID m_ThrustPID;
    common::PID m_RollPID;
    common::PID m_PitchPID;
    common::PID m_YawPID;
    std::ofstream m_Logfile;

    ignition::math::v4::Pose3d m_LoiterReference;
};
}
#endif