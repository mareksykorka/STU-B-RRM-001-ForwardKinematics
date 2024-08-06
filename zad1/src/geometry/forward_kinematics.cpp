#include "forward_kinematics.h"

ForwardKinematics::ForwardKinematics(ros::NodeHandle n)
{
    std::string robot_description;
    n.getParam("/robot_description", (std::string&)robot_description);
    if(robot_description.empty())
        throw std::invalid_argument("Cannot find parameter '/robot_description' on param server.\n "
                                    "This parameter is necessary for general function.\n"
                                    "It stores basic information about robot model and limits.");
    urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDF(robot_description);

    // Determine number of "single" joints
    for (auto i : robot_model->joints_)
    {
        int occurrences = 0;
        std::string::size_type pos = 0;
        std::string jointName = i.first;
        std::string target = "_";
        while ((pos = jointName.find(target, pos)) != std::string::npos)
        {
            ++occurrences;
            pos += target.length();
        }
        if(occurrences == 1) // joint_1 - there is only one "_"
            this->link_dimensions_.push_back(i.second->parent_to_joint_origin_transform.position.z); // get joint origin against parent.
    }

    publisher_tool_pose_ = n.advertise<geometry_msgs::Pose>("/tool_pose", 1);
    subscriber_joint_state_ = n.subscribe("/joint_states", 1, &ForwardKinematics::subscriberFunction, this);
}

Eigen::Matrix4d ForwardKinematics::rotationMatrix(char axis, double angle) const
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    switch(axis)
    {
        #pragma region Not Used
        //case 'x':
        //{
        //    break;
        //}
        #pragma endregion
        case 'y':
        {
            matrix << cos(angle), 0, sin(angle), 0,
                               0, 1,          0, 0,
                     -sin(angle), 0, cos(angle), 0,
                               0, 0,          0, 1;
            break;
        }
        case 'z':
        {
            matrix << cos(angle), -sin(angle), 0, 0,
                      sin(angle),  cos(angle), 0, 0,
                               0,           0, 1, 0,
                               0,           0, 0, 1;
            break;
        }
    }
    return matrix;
}

Eigen::Matrix4d ForwardKinematics::translationMatrix(char axis, double dist) const
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    switch(axis)
    {
        #pragma region Not Used
        //case 'x':
        //{
        //    break;
        //}
        //case 'y':
        //{
        //    break;
        //}
        #pragma endregion Not Used
        case 'z':
        {
            matrix << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, dist,
                      0, 0, 0, 1;
            break;
        }
    }

    return matrix;
}

void ForwardKinematics::subscriberFunction(const sensor_msgs::JointState::ConstPtr& msg)
{
    Eigen::Matrix4d matrix = translationMatrix('z', this->link_dimensions_[0]) * rotationMatrix('z', msg->position[0]) *
                             translationMatrix('z', this->link_dimensions_[1]) * rotationMatrix('y', msg->position[1]) *
                             translationMatrix('z', this->link_dimensions_[2]) * rotationMatrix('y', msg->position[2]) *
                             translationMatrix('z', this->link_dimensions_[3]) * rotationMatrix('z', msg->position[3]) *
                             translationMatrix('z', this->link_dimensions_[4]) * rotationMatrix('y', msg->position[4]) *
                             translationMatrix('z', this->link_dimensions_[5]) * translationMatrix('z', msg->position[5]);

    //Geometry pose message
    Eigen::Affine3d e_pose(matrix);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(e_pose,pose);
    publisher_tool_pose_.publish(pose);

    //FK transform
    tf::Transform transform;
    tf::poseEigenToTF (e_pose, transform);
    broadcaster_tf_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "tool0_calculated"));
}