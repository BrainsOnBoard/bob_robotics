#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {
/// \brief A plugin to control a simple cart with differential drive.
class DifferentialDrivePlugin : public ModelPlugin
{
public:
    /// \brief Constructor
    DifferentialDrivePlugin()
    {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0) {
            gzthrow("Invalid joint count, DifferentialDrivePlugin not loaded\n");
            return;
        }

        // Store the model pointer for convenience.
        m_Model = _model;

        // Get the first 2 joint. We are making an assumption about the model
        // having 2 joints which are the 2 wheels.
        m_LeftWheelJoint = m_Model->GetJoints()[1];
        m_RightWheelJoint = m_Model->GetJoints()[0];

        // Setup a P-controller, with a gain of 0.1.
        m_LeftWheelPID = common::PID(10, 0, 0);
        m_RightWheelPID = common::PID(10, 0, 0);

        // Apply the P-controller to the joint.
        m_Model->GetJointController()->SetVelocityPID(m_LeftWheelJoint->GetScopedName(), m_LeftWheelPID);
        m_Model->GetJointController()->SetVelocityPID(m_RightWheelJoint->GetScopedName(), m_RightWheelPID);

        // Default to zero velocity
        double velocity = 0;

        // Check that the velocity element exists, then read the value
        if (_sdf->HasElement("velocity"))
            velocity = _sdf->Get<double>("velocity");

        SetVelocity(velocity, velocity);

        // Create the m_Node
        m_Node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
        m_Node->Init(m_Model->GetWorld()->GetName());
#else
        m_Node->Init(m_Model->GetWorld()->Name());
#endif

        // Create a topic name
        std::string topicName = "~/" + m_Model->GetName() + "/vel_cmd";

        // Subscribe to the topic, and register a callback
        m_Sub = m_Node->Subscribe(topicName, &DifferentialDrivePlugin::OnMsg, this);
        std::cerr << "Subsribed to " << topicName << "\n";
    }

    /// \brief Set the velocity of the wheels in radians per second.
    /// \param[in] _vel New target velocity
    void SetVelocity(const double &_left_wheel_vel, const double &_right_wheel_vel)
    {
        // Set the joint's target velocity.
        m_Model->GetJointController()->SetVelocityTarget(m_LeftWheelJoint->GetScopedName(), -(_left_wheel_vel));
        m_Model->GetJointController()->SetVelocityTarget(m_RightWheelJoint->GetScopedName(), _right_wheel_vel);
        // std::cerr<< m_LeftWheelJoint->GetScopedName()<< ": " <<_left_wheel_vel << "." << m_RightWheelJoint->GetScopedName() << ": "<< _right_wheel_vel << std::endl;
    }

private:
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    void OnMsg(ConstVector2dPtr &_msg)
    {
        SetVelocity(_msg->x(), _msg->y());
    }

    /// \brief A node used for transport
    transport::NodePtr m_Node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr m_Sub;

    /// \brief Pointer to the model.
    physics::ModelPtr m_Model;

    /// \brief Pointer to the joint.
    physics::JointPtr m_LeftWheelJoint;
    physics::JointPtr m_RightWheelJoint;

    /// \brief A PID controller for the joint.
    common::PID m_LeftWheelPID;
    common::PID m_RightWheelPID;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(DifferentialDrivePlugin)
}
