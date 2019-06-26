#include <memory>

#include "JointStatePublisher.h"


using namespace gazebo;

void JoinStatePublisher::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    this->sdf = _sdf;
    bool publish_selected_only = false;
    std::string joint_names;
    if (sdf->HasElement("publish_selected_only"))
    {
        if (this->sdf->Get<bool>("publish_selected_only"))
        {
            publish_selected_only = true;
        }
    }
    if (publish_selected_only)
    {
        if (sdf->HasElement("joints"))
        {
            joint_names = sdf->Get<std::string>("joints");
        }
        else
        {
            sdf::ElementPtr child = sdf->GetFirstElement();
            while (child)
            {
                auto name = child->GetName();
                if (name == "joint")
                {
                    joint_names = joint_names + ", " + child->Get<std::string>();
                }
                child = child->GetNextElement();
            }
        }
    }
    if( publish_selected_only){
        DebugMessage("Publishing only the following joints:" + joint_names);
    }
    else {
        DebugMessage("Publishing all available joints.");

    }
    physics::Joint_V all_joints_vector = model->GetJoints();


    for (auto const &j : all_joints_vector)
    {

        std::string type = "moving";
        if (j->GetType() == (j->FIXED_JOINT | j->JOINT))
        {
            type = "fixed";
        }
        else
        {
            if (publish_selected_only)
            {
                if (joint_names.find(j->GetName()) == std::string::npos)
                {
                    continue;
                }
            }
            this->joints_vector.push_back(j);
        }
    }
    InitializeROSMembers();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JoinStatePublisher::OnUpdate, this));

    DebugMessage("Plugin is loaded");
}

void JoinStatePublisher::OnUpdate()
{
    ReadSimJointsData();
}

void JoinStatePublisher::InitializeROSMembers()
{
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "JSP", ros::init_options::NoSigintHandler);
        // DebugMessage("ROS Node is initialized manually.", WHITE_TXT);
    }
    this->rosNode_unique_ptr = std::make_unique<ros::NodeHandle>(this->model->GetName());
    this->rosPubJointStates = this->rosNode_unique_ptr->advertise<sensor_msgs::JointState>("joint_states", 10, false);
    this->timer = this->rosNode_unique_ptr->createTimer(ros::Duration(1.0 / 50.), &JoinStatePublisher::timerCallback,
                                                        this);    // TODO: Change rate later


    this->msg_joint_state = sensor_msgs::JointState();

    this->num_joints = this->joints_vector.size();

    this->msg_joint_state.position.resize(this->num_joints);
    this->msg_joint_state.velocity.resize(this->num_joints);
    this->msg_joint_state.effort.resize(this->num_joints);

    this->j_pos.resize(num_joints);
    this->j_rate.resize(num_joints);
    this->j_effort.resize(num_joints);

    for (auto const &j : joints_vector)
    {
        this->msg_joint_state.name.push_back(j->GetName());
    }
}

void JoinStatePublisher::timerCallback(const ros::TimerEvent &event)
{
    for (int i = 0; i < num_joints; i++)
    {
        this->msg_joint_state.position[i] = this->j_pos.at(i);
        this->msg_joint_state.velocity[i] = this->j_rate.at(i);
        this->msg_joint_state.effort[i] = this->j_effort.at(i);
    }
    msg_joint_state.header.stamp = ros::Time::now();
    this->rosPubJointStates.publish(this->msg_joint_state);
}

void JoinStatePublisher::ReadSimJointsData()
{
    for (int i = 0; i < num_joints; i++)
    {
        this->j_pos.at(i) = GetJointPosition(joints_vector.at(i));
        this->j_effort.at(i) = joints_vector.at(i)->GetForce(0);
        this->j_rate.at(i) = alpha * joints_vector.at(i)->GetVelocity(0) + (1 - alpha) * this->j_rate.at(i);
    }
}

double JoinStatePublisher::GetJointPosition(const physics::JointPtr joint, const double axis_index)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return joint->Position(axis_index);
#else
    return joint->GetAngle(axis_index).Radian();
#endif
}
