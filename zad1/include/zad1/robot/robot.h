#ifndef CATKIN_PRVE_ZADANIE_ROBOT_H
#define CATKIN_PRVE_ZADANIE_ROBOT_H

#include <ros/ros.h>
#include <urdf_parser/urdf_parser.h>
#include <stdexcept>
#include <vector>

class Robot
{
    public:
        Robot() = delete;
        Robot(ros::NodeHandle n);

        std::vector<double> getCurrentPositions() const;
        double getCurrentPosition(int joint_id) const;

        int getJointCount() const;
        double getJointUpperLimit(int joint_id) const;
        double getJointLowerLimit(int joint_id) const;

        void move(const std::vector<double> &new_positions);
        void move(int joint_id, double joint_value);
private:
        int joint_count_;
        std::vector<double> positions_;
        urdf::ModelInterfaceSharedPtr robot_model_;
};

#endif //CATKIN_PRVE_ZADANIE_ROBOT_H