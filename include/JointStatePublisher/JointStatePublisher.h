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


#define BLUE_TXT1 "\e[1;34m"
#define NO_COLOR "\033[0m"


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

        // ROLL, PITCH, YAW, PAN, TILT and corresponding rates
        double r,p,y,rr,pr,yr, pan, pan_r, tilt, tilt_r;

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


        void DebugMessage(const std::string Message, const std::string color = BLUE_TXT1, const bool set_no_color = true) {
            std::string st = color + "[JointsPublisher] " +Message + NO_COLOR;
            ROS_INFO("%s", st.c_str());
        }

        void DebugMessageERROR(const std::string Message) {
            ROS_ERROR("%s", Message.c_str());
        }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JoinStatePublisher)
}

