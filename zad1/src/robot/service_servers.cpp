#include "service_servers.h"

ServiceServers::ServiceServers(ros::NodeHandle n, Robot *robot, std::mutex *mtx)
{
    this->robot_ = robot;
    this->mtx_lock_ = mtx;
    this->service_absolute_ = n.advertiseService("/move_absolute", &ServiceServers::serviceAbsoluteFunction, this);
    this->service_relative_ = n.advertiseService("/move_relative", &ServiceServers::serviceRelativeFunction, this);
}

bool ServiceServers::inLimits(int joint_id, double joint_value) const
{
    return (this->robot_->getJointLowerLimit(joint_id) <= joint_value+FLT_EPSILON) && (joint_value-FLT_EPSILON <= this->robot_->getJointUpperLimit(joint_id));
}

bool ServiceServers::serviceAbsoluteFunction(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res)
{
    this->mtx_lock_->lock(); // Lock robot access mutex
    try
    {
        if(req.positions.size() != this->robot_->getJointCount())
            throw std::out_of_range("Input array does not match robot joint count.");

        for (int i = 0; i < this->robot_->getJointCount(); i++)
        {
            if(!ServiceServers::inLimits(i, req.positions[i]))
                throw std::out_of_range("Requested position for Joint '" + std::to_string(i+1) + "' is outside it's limits '" +
                                        std::to_string(this->robot_->getJointLowerLimit(i)) + " <= " +
                                        std::to_string(req.positions[i]) + " <= " +
                                        std::to_string(this->robot_->getJointUpperLimit(i)) + "'.");
        }

        this->robot_->move(req.positions);

        res.success = true;
        res.message = "Robot joints moved to ABSOLUTE position.";
    }
    catch (std::exception &e)
    {
        res.success = false;
        res.message = "Something went wrong, I catch exception: " + std::string(e.what());
    }
    this->mtx_lock_->unlock(); // Unlock robot access mutex
    return true;
}

bool ServiceServers::serviceRelativeFunction(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res)
{
    this->mtx_lock_->lock(); // Lock robot access mutex
    try
    {
        if(req.positions.size() != this->robot_->getJointCount())
            throw std::out_of_range("Input array does not match robot joint count.");

        for (int i = 0; i < this->robot_->getJointCount(); i++)
        {
            if(!ServiceServers::inLimits(i, this->robot_->getCurrentPosition(i) + req.positions[i]))
                throw std::out_of_range("Requested position for Joint '" + std::to_string(i+1) + "' is outside it's limits '" +
                                        std::to_string(this->robot_->getJointLowerLimit(i)) + " <= " +
                                        std::to_string(this->robot_->getCurrentPosition(i) + req.positions[i]) + " <= " +
                                        std::to_string(this->robot_->getJointUpperLimit(i)) + "'.");
        }

        for (int i = 0; i < this->robot_->getJointCount(); i++)
            this->robot_->move(i, this->robot_->getCurrentPosition(i) + req.positions[i]);

        res.success = true;
        res.message = "Robot joints moved RELATIVE to previous position.";
    }
    catch (std::exception &e)
    {
        res.success = false;
        res.message = "Something went wrong, I catch exception: " + std::string(e.what());
    }
    this->mtx_lock_->unlock(); // Unlock robot access mutex
    return true;
}