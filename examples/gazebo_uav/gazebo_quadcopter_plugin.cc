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

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include "gazebo_quadcopter_plugin.h"

// Gazebo includes
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
// Third-party includes
#include <ignition/math/Filter.hh>
#include <ignition/math/Vector3.hh>
// Standard C++ includes
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <sdf/sdf.hh>
#include <string>
#include <vector>
#define MAX_MOTORS 4

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboQuadCopterPlugin)

/// \brief Obtains a parameter from sdf.
/// \param[in] _sdf Pointer to the sdf object.
/// \param[in] _name Name of the parameter.
/// \param[out] _param Param Variable to write the parameter to.
/// \param[in] _default_value Default value, if the parameter not available.
/// \param[in] _verbose If true, gzerror if the parameter is not available.
/// \return True if the parameter was found in _sdf, false otherwise.
template <class T>
void
getSdfParam(sdf::ElementPtr _sdf, const std::string &_name, T &_param, const T &_defaultValue, const bool &_verbose = false)
{
    if (_sdf->HasElement(_name)) {
        _param = _sdf->GetElement(_name)->Get<T>();
        return;
    }

    _param = _defaultValue;
    if (_verbose) {
        gzerr << "[GazeboQuadCopterPlugin] Please specify a value for parameter ["
              << _name << "].\n";
    }
}

/////////////////////////////////////////////////
void
GazeboQuadCopterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "GazeboQuadCopterPlugin _model pointer is null");
    GZ_ASSERT(_sdf, "GazeboQuadCopterPlugin _sdf pointer is null");

    m_Model = _model;

    // per rotor
    if (_sdf->HasElement("rotor")) {
        sdf::ElementPtr rotorSDF = _sdf->GetElement("rotor");

        while (rotorSDF) {
            Rotor rotor;

            if (rotorSDF->HasAttribute("id")) {
                rotor.m_Id = rotorSDF->GetAttribute("id")->Get(rotor.m_Id);
            } else {
                rotor.m_Id = m_Rotors.size();
                gzwarn << "id attribute not specified, use order parsed ["
                       << rotor.m_Id << "].\n";
            }

            if (rotorSDF->HasElement("jointName")) {
                rotor.m_JointName = rotorSDF->Get<std::string>("jointName");
            } else {
                gzerr << "Please specify a jointName,"
                      << " where the rotor is attached.\n";
            }

            // Get the pointer to the joint.
            rotor.m_Joint = _model->GetJoint(rotor.m_JointName);
            if (rotor.m_Joint == nullptr) {
                gzerr << "Couldn't find specified joint ["
                      << rotor.m_JointName << "]. This plugin will not run.\n";
                return;
            }

            if (rotorSDF->HasElement("turningDirection")) {
                std::string turningDirection = rotorSDF->Get<std::string>(
                        "turningDirection");
                // special cases mimic from rotors_gazebo_plugins
                if (turningDirection == "cw")
                    rotor.m_Multiplier = -1;
                else if (turningDirection == "ccw")
                    rotor.m_Multiplier = 1;
                else {
                    gzdbg << "not string, check turningDirection as float\n";
                    rotor.m_Multiplier = rotorSDF->Get<double>("turningDirection");
                }
            } else {
                rotor.m_Multiplier = 1;
                gzerr << "Please specify a turning"
                      << " direction multiplier ('cw' or 'ccw'). Default 'ccw'.\n";
            }

            getSdfParam<double>(rotorSDF, "rotorVelocitySlowdownSim", rotor.m_RotorVelocitySlowdownSim, 1);

            if (ignition::math::equal(rotor.m_RotorVelocitySlowdownSim, 0.0)) {
                gzerr << "rotor for joint [" << rotor.m_JointName
                      << "] rotorVelocitySlowdownSim is zero,"
                      << " aborting plugin.\n";
                return;
            }

            // Overload the PID parameters if they are available.
            double param;
            getSdfParam<double>(rotorSDF, "vel_p_gain", param, rotor.m_Pid.GetPGain());
            rotor.m_Pid.SetPGain(param);

            getSdfParam<double>(rotorSDF, "vel_i_gain", param, rotor.m_Pid.GetIGain());
            rotor.m_Pid.SetIGain(param);

            getSdfParam<double>(rotorSDF, "vel_d_gain", param, rotor.m_Pid.GetDGain());
            rotor.m_Pid.SetDGain(param);

            getSdfParam<double>(rotorSDF, "vel_i_max", param, rotor.m_Pid.GetIMax());
            rotor.m_Pid.SetIMax(param);

            getSdfParam<double>(rotorSDF, "vel_i_min", param, rotor.m_Pid.GetIMin());
            rotor.m_Pid.SetIMin(param);

            getSdfParam<double>(rotorSDF, "vel_cmd_max", param, rotor.m_Pid.GetCmdMax());
            rotor.m_Pid.SetCmdMax(param);

            getSdfParam<double>(rotorSDF, "vel_cmd_min", param, rotor.m_Pid.GetCmdMin());
            rotor.m_Pid.SetCmdMin(param);

            // set pid initial command
            rotor.m_Pid.SetCmd(0);

            m_Rotors.push_back(rotor);
            rotorSDF = rotorSDF->GetNextElement("rotor");
        }
    }
    // Get sensors
    std::string imuName;
    getSdfParam<std::string>(_sdf, "imuName", imuName, "imu_sensor");
    std::string imuScopedName = m_Model->GetWorld()->Name() + "::" + m_Model->GetScopedName() + "::" + imuName;
    m_ImuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor(imuScopedName));

    if (!m_ImuSensor) {
        gzerr << "imu_sensor [" << imuScopedName
              << "] not found, abort GazeboQuadCopter plugin.\n"
              << "\n";
        return;
    }

    // Controller time control.
    m_LastControllerUpdateTime = 0;

    // Create the Node
    m_Node = transport::NodePtr(new transport::Node());
    m_Node->Init(_model->GetWorld()->Name());

    // Create a topic name
    const std::string topicName = "~/gazebo_quadcopter/motors_cmd";

    // Subscribe to the topic, and register a callback
    m_Sub = m_Node->Subscribe(topicName, &GazeboQuadCopterPlugin::OnMsg, this);
    std::cerr << "Subsribed to " << topicName << "\n";
    // Listen to the update event. This event is broadcast every simulation iteration.
    m_UpdateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboQuadCopterPlugin::OnUpdate, this));
    std::cout << "GazeboQuadCopter ready to fly. The force will be with you" << std::endl;

    // Altitude hold PID Controller
    sdf::ElementPtr altholdSDF = _sdf->GetElement("althold");
    double param;
    getSdfParam<double>(altholdSDF, "p_gain", param, 0);
    m_ThrustPID.SetPGain(param);

    getSdfParam<double>(altholdSDF, "i_gain", param, 0);
    m_ThrustPID.SetIGain(param);

    getSdfParam<double>(altholdSDF, "d_gain", param, 0);
    m_ThrustPID.SetDGain(param);

    getSdfParam<double>(altholdSDF, "i_max", param, 0);
    m_ThrustPID.SetIMax(param);

    getSdfParam<double>(altholdSDF, "i_min", param, 0);
    m_ThrustPID.SetIMin(param);

    getSdfParam<double>(altholdSDF, "cmd_max", param, 0);
    m_ThrustPID.SetCmdMax(param);

    getSdfParam<double>(altholdSDF, "cmd_min", param, 0);
    m_ThrustPID.SetCmdMin(param);

    // set pid initial command
    m_ThrustPID.SetCmd(0);

    sdf::ElementPtr loiterSDF = _sdf->GetElement("loiter");
    getSdfParam<double>(loiterSDF, "p_gain", param, 0);
    m_RollPID.SetPGain(param);
    m_PitchPID.SetPGain(param);
    m_YawPID.SetPGain(param);

    getSdfParam<double>(loiterSDF, "i_gain", param, 0);
    m_RollPID.SetIGain(param);
    m_PitchPID.SetIGain(param);
    m_YawPID.SetIGain(param);

    getSdfParam<double>(loiterSDF, "d_gain", param, 0);
    m_RollPID.SetDGain(param);
    m_PitchPID.SetDGain(param);
    m_YawPID.SetDGain(param);

    getSdfParam<double>(loiterSDF, "i_max", param, 0);
    m_RollPID.SetIMax(param);
    m_PitchPID.SetIMax(param);
    m_YawPID.SetIMax(param);

    getSdfParam<double>(loiterSDF, "i_min", param, 0);
    m_RollPID.SetIMin(param);
    m_PitchPID.SetIMin(param);
    m_YawPID.SetIMin(param);

    getSdfParam<double>(loiterSDF, "cmd_max", param, 0);
    m_RollPID.SetCmdMax(param);
    m_PitchPID.SetCmdMax(param);
    m_YawPID.SetCmdMax(param);

    getSdfParam<double>(loiterSDF, "cmd_min", param, 0);
    m_RollPID.SetCmdMin(param);
    m_PitchPID.SetCmdMin(param);
    m_YawPID.SetCmdMin(param);

    m_RollPID.SetCmd(0);
    m_PitchPID.SetCmd(0);
    m_YawPID.SetCmd(0);

    const char *logfile_location = std::getenv("LOG_FILE");
    if (logfile_location != nullptr) {
        m_Logfile.open(logfile_location, std::ios_base::app);
        GZ_ASSERT(m_Logfile.good(), "Log file cannot be opened.\n");
    }
    else{
        std::cerr<< "LOG_FILE not set. Flight log is disabled.\n";
    }
}

/////////////////////////////////////////////////
void
GazeboQuadCopterPlugin::OnMsg(ConstQuaternionPtr &_msg)
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_Thrust = _msg->w();
    m_Roll = _msg->x();
    m_Pitch = _msg->y();
    m_Yaw = _msg->z();
}
void
GazeboQuadCopterPlugin::MotorMixing(const double _dt)
{
    float thrust = 0, roll = 0, pitch = 0, yaw = 0;
    float fr = 0, fl = 0, br = 0, bl = 0;
    double z, r, p;
    ignition::math::Pose3d actualPose, referencePose;
    actualPose = m_Model->WorldPose();
    referencePose = m_LoiterReference;
    z = actualPose.Pos().Z();
    r = actualPose.Rot().Roll();
    p = actualPose.Rot().Pitch();

    if (0.49 < m_Thrust && m_Thrust < 0.51) {
        thrust = m_ThrustPID.Update(z - referencePose.Pos().Z(), _dt);
    } else {
        thrust = m_Thrust;
        m_LoiterReference.Set(referencePose.Pos().X(), referencePose.Pos().Y(), z, referencePose.Rot().Roll(), referencePose.Rot().Pitch(), referencePose.Rot().Yaw());
    }

    roll = m_RollPID.Update(r - m_Roll, _dt);    //angular displacement PID
    pitch = m_PitchPID.Update(p - m_Pitch, _dt); // angular displacement PID
    yaw = m_Yaw * 0.1;                           //angular velocity PID

    if (m_Logfile.is_open()) {
        m_Logfile << "thrust: " << thrust << "\troll: " << roll << "\tpitch: " << pitch << "\tyaw: " << yaw << "\t";
        m_Logfile << "\tpos: " << actualPose.Pos() << " \trot: " << actualPose.Rot() << std::endl;
    }
    fr = thrust + yaw - pitch - roll;
    fl = thrust - yaw - pitch + roll;
    br = thrust - yaw + pitch - roll;
    bl = thrust + yaw + pitch + roll;

    m_Rotors[0].m_Cmd = m_Rotors[0].m_MaxRpm * fr;
    m_Rotors[1].m_Cmd = m_Rotors[1].m_MaxRpm * bl;
    m_Rotors[2].m_Cmd = m_Rotors[2].m_MaxRpm * fl;
    m_Rotors[3].m_Cmd = m_Rotors[3].m_MaxRpm * br;
}
/////////////////////////////////////////////////
void
GazeboQuadCopterPlugin::OnUpdate()
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    gazebo::common::Time curTime = m_Model->GetWorld()->SimTime();
    ApplyMotorForces((curTime - m_LastControllerUpdateTime).Double());
    m_LastControllerUpdateTime = curTime;
}
/////////////////////////////////////////////////
void
GazeboQuadCopterPlugin::ResetPIDs()
{
    // Reset velocity PID for rotors
    for (size_t i = 0; i < m_Rotors.size(); ++i) {
        m_Rotors[i].m_Cmd = 0;
    }
}

/////////////////////////////////////////////////
void
GazeboQuadCopterPlugin::ApplyMotorForces(const double _dt)
{
    MotorMixing(_dt);
    // update velocity PID for rotors and apply force to joint
    for (size_t i = 0; i < m_Rotors.size(); ++i) {
        const double velTarget = m_Rotors[i].m_Multiplier *
                           m_Rotors[i].m_Cmd /
                           m_Rotors[i].m_RotorVelocitySlowdownSim;
        const double vel = m_Rotors[i].m_Joint->GetVelocity(0);
        const double error = vel - velTarget;
        const double force = m_Rotors[i].m_Pid.Update(error, _dt);
        m_Rotors[i].m_Joint->SetForce(0, force);
    }
}
