#include "robot.h"

Robot::Robot(ros::NodeHandle n)
{
    // Read robot model description and store it in "urdf::ModelInterfaceSharedPtr robot_model_"
    this->joint_count_ = 0;
    std::string robot_description;
    n.getParam("/robot_description", (std::string&)robot_description);
    if(robot_description.empty())
        throw std::invalid_argument("Cannot find parameter '/robot_description' on param server.\n "
                                    "This parameter is necessary for general function.\n"
                                    "It stores basic information about robot model and limits.");
    this->robot_model_ = urdf::parseURDF(robot_description);

    // Determine number of "single" joints
    for (auto i : robot_model_->joints_)
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
            this->joint_count_++;
    }

    if (this->joint_count_ <= 0)
        throw std::invalid_argument("Invalid argument 'number_of_joints'.\n Value '" + std::to_string(this->joint_count_) +
                                    "' must be non-zero and non-negative.");
    this->positions_.resize(this->joint_count_);
}

std::vector<double> Robot::getCurrentPositions() const
{
    return this->positions_;
}

double Robot::getCurrentPosition(int joint_id) const
{
    if(joint_id < 0 || joint_id > this->joint_count_)
        throw std::out_of_range("Argument 'joint_id' value '" + std::to_string(joint_id) +
                                "' is out of range '" + std::to_string(joint_count_) + "'.");
    else
        return this->positions_[joint_id];
}

int Robot::getJointCount() const
{
    return this->positions_.size();
}

double Robot::getJointUpperLimit(int joint_id) const
{
    if(joint_id < 0 || joint_id > this->joint_count_)
        throw std::out_of_range("Argument 'joint_id' value '" + std::to_string(joint_id) +
                                "' is out of range '" + std::to_string(joint_count_) + "'.");
    else
        return this->robot_model_->joints_["joint_"+std::to_string(joint_id+1)]->limits->upper;
}

double Robot::getJointLowerLimit(int joint_id) const
{
    if(joint_id < 0 || joint_id > this->joint_count_)
        throw std::out_of_range("Argument 'joint_id' value '" + std::to_string(joint_id) +
                                "' is out of range '" + std::to_string(joint_count_) + "'.");
    else
        return this->robot_model_->joints_["joint_"+std::to_string(joint_id+1)]->limits->lower;
}

void Robot::move(const std::vector<double> &new_positions)
{
    if(new_positions.size() != this->joint_count_)
        throw std::invalid_argument("Argument 'new_positions' is not the same size.");
    else
        this->positions_ = new_positions;
}

void Robot::move(int joint_id, double joint_value)
{
    if(joint_id < 0 || joint_id > this->joint_count_)
        throw std::out_of_range("Argument 'joint_id' value '" + std::to_string(joint_id) +
                                "' is out of range '" + std::to_string(joint_count_) + "'.");
    else
        this->positions_[joint_id] = joint_value;
}