#ifndef _DIFFERENTIAL_DRIVE_PLUGIN_HH_
#define _DIFFERENTIAL_DRIVE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief A plugin to control a simple cart with differential drive.
  class DifferentialDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: DifferentialDrivePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, DifferentialDrivePlugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first 2 joint. We are making an assumption about the model
      // having 2 joints which are the 2 wheels.
      this->left_wheel_joint = _model->GetJoints()[1];
      this->right_wheel_joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
      this->left_wheel_pid = common::PID(10, 0, 0);
      this->right_wheel_pid = common::PID(10, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(this->left_wheel_joint->GetScopedName(), this->left_wheel_pid);
      this->model->GetJointController()->SetVelocityPID(this->right_wheel_joint->GetScopedName(), this->right_wheel_pid);

      // Default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity, velocity);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName, &DifferentialDrivePlugin::OnMsg, this);
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_left_wheel_vel, const double &_right_wheel_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(this->left_wheel_joint->GetScopedName(), -(_left_wheel_vel));
      this->model->GetJointController()->SetVelocityTarget(this->right_wheel_joint->GetScopedName(), _right_wheel_vel);
      // std::cerr<< this->left_wheel_joint->GetScopedName()<< ": " <<_left_wheel_vel << "." << this->right_wheel_joint->GetScopedName() << ": "<< _right_wheel_vel << std::endl;
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x(), _msg->y());
    }

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr left_wheel_joint;
    private: physics::JointPtr right_wheel_joint;

    /// \brief A PID controller for the joint.
    private: common::PID left_wheel_pid;
    private: common::PID right_wheel_pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DifferentialDrivePlugin)
}
#endif