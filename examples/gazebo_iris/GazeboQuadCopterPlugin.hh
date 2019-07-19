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

using namespace gazebo;
/// \brief Rotor class
class Rotor
{
    /// \brief Constructor
public:
    Rotor()
    {
        // most of these coefficients are not used yet.
        this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
        this->frequencyCutoff = this->kDefaultFrequencyCutoff;
        this->samplingRate = this->kDefaultSamplingRate;

        this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
    }

    /// \brief rotor id
    int id = 0;

    /// \brief Max rotor propeller RPM.
    double maxRpm = 838.0;

    /// \brief Next command to be applied to the propeller
    double cmd = 0;

    /// \brief Velocity PID for motor control
    common::PID pid;

    /// \brief Control propeller joint.
    std::string jointName;

    /// \brief Control propeller joint.
    physics::JointPtr joint;

    /// \brief direction multiplier for this rotor
    double multiplier = 1;

    /// \brief unused coefficients
    double rotorVelocitySlowdownSim;

    double frequencyCutoff;

    double samplingRate;

    ignition::math::OnePole<double> velocityFilter;

    static double kDefaultRotorVelocitySlowdownSim;

    static double kDefaultFrequencyCutoff;

    static double kDefaultSamplingRate;
};

namespace gazebo {
// Forward declare private data class
class GazeboQuadCopterPluginPrivate
{
public:
    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model;
    physics::ModelPtr model;

    /// \brief array of propellers
    std::vector<Rotor> rotors;

    /// \brief keep track of controller update sim-time.
    gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
    std::mutex mutex;

    /// \brief Socket handle
    int handle;

    /// \brief Pointer to an IMU sensor
    sensors::ImuSensorPtr imuSensor;

    /// \brief number of times GazeboQuadCotper skips update
    int connectionTimeoutCount;

    /// \brief number of times GazeboQuadCotper skips update
    /// before marking GazeboQuadCopter offline
    int connectionTimeoutMaxCount;

    double m_Thrust; double m_Roll; double m_Pitch; double m_Yaw;
};

class GAZEBO_VISIBLE GazeboQuadCopterPlugin : public ModelPlugin
{
    /// \brief Constructor.
public:
    GazeboQuadCopterPlugin();

    /// \brief Destructor.
    ~GazeboQuadCopterPlugin();

    // Documentation Inherited.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
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

    /// \brief Private data pointer.
    std::unique_ptr<GazeboQuadCopterPluginPrivate> dataPtr;

    /// \brief A node used for transport
    transport::NodePtr m_Node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr m_Sub;


    common::PID thrustPID;
    common::PID rollPID;
    common::PID pitchPID;
    common::PID yawPID;

    double altitudeReference;
    ignition::math::v4::Pose3d loiterReference;
};
}
#endif