//RCCar_plugin

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>


namespace gazebo {
/// \brief A plugin to control a simple car with ackermann drive using the keyboard [w,a,s,d]
// It needs KeyboardGUIPlugin to run, which can be enabled in the .world file
class RCCarPlugin : public ModelPlugin
{
public:
    /// \brief Constructor
    RCCarPlugin()
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
            gzthrow("Invalid joint count, model not loaded\n");
            return;
        }

        // the model
        m_Model = _model;

        // get joints by name from sdf file
        m_FrontLeftJoint = m_Model->GetJoint("wheel_front_left_steer_spin");
        m_FrontRightJoint = m_Model->GetJoint("wheel_front_right_steer_spin");
        m_RearLeftJoint = m_Model->GetJoint("wheel_rear_left_spin");
        m_RearRightJoint = m_Model->GetJoint("wheel_rear_right_spin");

        // Setup a P-controller, with a gain of 0.1.
        m_WheelPID = common::PID(10, 0, 0);

        // Apply the P-controller to the joints on the wheels
        m_Model->GetJointController()->SetVelocityPID(m_RearLeftJoint->GetScopedName(), m_WheelPID);
        m_Model->GetJointController()->SetVelocityPID(m_RearRightJoint->GetScopedName(), m_WheelPID);
        m_Model->GetJointController()->SetVelocityPID(m_FrontLeftJoint->GetScopedName(), m_WheelPID);
        m_Model->GetJointController()->SetVelocityPID(m_FrontRightJoint->GetScopedName(), m_WheelPID);

        // Default to zero velocity
        double velocity = 0;

        // Check that the velocity element exists, then read the value
        if (_sdf->HasElement("velocity"))
            velocity = _sdf->Get<double>("velocity");

        SetVelocity(velocity, 0);

        // Create the m_Node
        m_Node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
        m_Node->Init(m_Model->GetWorld()->GetName());
#else
        m_Node->Init(m_Model->GetWorld()->Name());
#endif

        // topic names
        std::string topicName = "~/" + m_Model->GetName() + "/vel_cmd";
        std::string keypressTopicName = "~/keyboard/keypress";

        // Subscribe to the topic, and register a callback
        m_Sub = m_Node->Subscribe(topicName, &RCCarPlugin::OnMsg, this);
        std::cerr << "Subsribed to " << topicName << "\n";

        // subscribe to key events from gazebo gui for keyboard control
        m_SubKeyPress = m_Node->Subscribe(keypressTopicName, &RCCarPlugin::OnKeyMsg, this);
        std::cerr << "Subsribed to " << keypressTopicName << "\n";

        // start update listening - will call OnUpdate() on update
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RCCarPlugin::OnUpdate, this));

        // publish coordinates to /[model_name]/position topic
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        pub = node->Advertise<gazebo::msgs::Pose>("~/"  + m_Model->GetName() + "/pose");

    }

    /// \brief Set the velocity of the wheels in radians per second.
    /// \param[in] _vel New target velocity
    void SetVelocity(const double &speed, const double &turning)
    {
        // Set the rear wheel joints target velocity.
        m_Model->GetJointController()->SetVelocityTarget(m_RearLeftJoint->GetScopedName(), speed);
        m_Model->GetJointController()->SetVelocityTarget(m_RearRightJoint->GetScopedName(), speed);
        SetSteering(turning);
    }

    /// \brief Set the steering angle.
    /// \param[in] _vel steering angle
    void SetSteering(const double turning_angle) {
        double ang1,ang2;
        getAckermannAngles(turning_angle,ang1,ang2);
        m_FrontLeftJoint->SetPosition(0,ang1);
        m_FrontRightJoint->SetPosition(0,ang2);
    }

private:
    /// \brief Handle incoming message
    /// \param[in] _msg a vector containing velocity and steering angle
    void OnMsg(ConstVector2dPtr &_msg)
    {
        SetVelocity(_msg->x(), _msg->y());
    }

    /// \brief handle keyboard messages - control the robot
    /// \param[in] _msg a string holding the key message, keypress is stored in _msg[3]
    void OnKeyMsg(const std::string &_msg)
    {
        double ang1,ang2;
        if (_msg[3] == 'd') {
            // turn left 30 degrees
            SetSteering(-0.523599);
        }

        else if (_msg[3] == 'a') {
            // turn right 30 degrees
            SetSteering(0.523599);
        }

        // go forward
        if (_msg[3] == 'w') {
            SetVelocity(7.0, 0.0);
        }

        // stop motors
        else if (_msg[3] == 'x') {
            SetVelocity(0.0, 0.0);
        }

        // reverse
        else if (_msg[3] == 'c') {
            SetVelocity(-7.0, 0.0);
        }
    }

    /// \brief Called on every update of the simulation
    void OnUpdate()
    {
        // get robot position
        auto pose = m_Model->WorldPose();
        gazebo::msgs::Set(&msg, pose);
        //publish robot Pose every update
        pub->Publish(msg);
    }


    /// \brief calculates the proper angles for both wheels to minimise slippage.
    void getAckermannAngles(const double angle, double &angle_left, double &angle_right) {
        double numerator = 2.0 * wheelBase * sin(angle);
        angle_left = atan2(numerator,(2.0*wheelBase*cos(angle) - wheelSeparation*sin(angle)) );
        angle_right = atan2(numerator,(2.0*wheelBase*cos(angle) + wheelSeparation*sin(angle)) );
    }

    /// \brief a pointer to a publisher to transport Pose messages
    gazebo::transport::PublisherPtr pub;

    /// \brief A node used for transport
    transport::NodePtr m_Node;

    /// \brief A subscriber to a named topic.
    transport::SubscriberPtr m_Sub;

    /// \brief A subscriber for keypress events
    transport::SubscriberPtr m_SubKeyPress;

    /// \brief Pointer to the model.
    physics::ModelPtr m_Model;

    /// \brief rear joints
    physics::JointPtr m_RearLeftJoint;
    physics::JointPtr m_RearRightJoint;

    /// \brief front joints
    physics::JointPtr m_FrontLeftJoint;
    physics::JointPtr m_FrontRightJoint;

    /// \brief A PID controller for the joint.
    common::PID m_WheelPID;

    /// \brief a pointer to the update event
    event::ConnectionPtr updateConnection;

    /// \brief Robot pose to publish
    gazebo::msgs::Pose msg;

    /// \brief robot parameters, change this according to the robot spec
    const double wheelBase = 1.0;
    const double wheelSeparation = 0.5;

};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RCCarPlugin)
}
