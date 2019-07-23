// BoB robotics includes
#include "common/circstat.h"
#include "common/macros.h"
#include "robots/gazebo/omni2d.h"

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

using namespace BoBRobotics;
using namespace units::velocity;

namespace gazebo {
/// \brief A plugin to control a simple cart with differential drive.
class OmniWheelPlugin : public ModelPlugin
{
public:
    /// \brief Constructor
    OmniWheelPlugin()
    {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr)
    {
        // Safety check
        BOB_ASSERT(model->GetJointCount() == 3);

        // Store the model pointer for convenience.
        m_Model = model;

        // Get the first 2 joint. We are making an assumption about the model
        // having 2 joints which are the 2 wheels.
        m_LeftJoint = m_Model->GetJoint("left_joint");
        m_RightJoint = m_Model->GetJoint("right_joint");
        m_BackJoint = m_Model->GetJoint("back_joint");

        // Create the Node
        m_Node = transport::NodePtr(new transport::Node());
        m_Node->Init(m_Model->GetWorld()->Name());

        // Subscribe to the topic, and register a callback
        m_Sub = m_Node->Subscribe(Robots::Gazebo::Omni2D::TopicName,
                                  &OmniWheelPlugin::OnMsg,
                                  this);
        std::cout << "Subscribed to " << Robots::Gazebo::Omni2D::TopicName << std::endl;
    }

    /// \brief Set the velocity of the wheels in radians per second.
    /// \param[in] _vel New target velocity
    void setVelocity(const double &left, const double &right, const double &back)
    {
        // std::lock_guard<std::mutex> lock(m_VelocitiesMutex);
        auto setWheelVelocity = [](auto &joint, const double &value) {
            joint.SetVelocity(0, getAngularVelocity(meters_per_second_t{ value },
                                                    Robots::Gazebo::Omni2D::WheelRadius).value());
        };

        setWheelVelocity(*m_LeftJoint, left);
        setWheelVelocity(*m_RightJoint, right);
        setWheelVelocity(*m_BackJoint, back);
    }

private:
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    void OnMsg(ConstVector3dPtr &_msg)
    {
        setVelocity(_msg->x(), _msg->y(), _msg->z());
    }

    /// \brief A node used for transport
    transport::NodePtr m_Node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr m_Sub;

    gazebo::event::ConnectionPtr m_Connection;

    /// \brief Pointer to the model.
    physics::ModelPtr m_Model;

    /// \brief Pointer to the joints.
    physics::JointPtr m_LeftJoint, m_RightJoint, m_BackJoint;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(OmniWheelPlugin)
}
