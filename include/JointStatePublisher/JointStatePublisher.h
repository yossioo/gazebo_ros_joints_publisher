#include <iostream>
#include <string>
#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
    class JoinStatePublisher : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:
        physics::ModelPtr model;
        physics::Joint_V joints_vector;
        sdf::ElementPtr sdf;
        event::ConnectionPtr updateConnection;

        int num_joints = 0;
        std::vector<double> j_pos, j_rate, j_effort;
        double alpha = 0.01;

        // ROS members
        std::unique_ptr<ros::NodeHandle> rosNode_unique_ptr;
        // ros::NodeHandle *node_handle_;
        ros::Publisher rosPubJointStates, rosPubJointStates2;
        ros::Timer timer;
        sensor_msgs::JointState msg_joint_state;

        void InitializeROSMembers();
        void timerCallback(const ros::TimerEvent &event);
        void ReadSimJointsData();
        double GetJointPosition(const physics::JointPtr joint, const double axis_index = 0);
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JoinStatePublisher)
}

