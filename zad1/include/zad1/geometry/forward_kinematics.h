#ifndef CATKIN_PRVE_ZADANIE_FORWARD_KINEMATICS_H
#define CATKIN_PRVE_ZADANIE_FORWARD_KINEMATICS_H
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf_parser/urdf_parser.h>

class ForwardKinematics
{
    public:
        ForwardKinematics() = delete;
        ForwardKinematics(ros::NodeHandle n);
        Eigen::Matrix4d rotationMatrix(char axis, double angle) const;
        Eigen::Matrix4d translationMatrix(char axis, double dist) const;
    private:
        std::vector<double> link_dimensions_;
        ros::Subscriber subscriber_joint_state_;
        ros::Publisher publisher_tool_pose_;
        tf::TransformBroadcaster broadcaster_tf_;
        void subscriberFunction(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif //CATKIN_PRVE_ZADANIE_FORWARD_KINEMATICS_H
