#ifndef CATKIN_PRVE_ZADANIE_SERVICESERVER_H
#define CATKIN_PRVE_ZADANIE_SERVICESERVER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <rrm_msgs/Move.h>
#include "robot.h"
#include "float.h"

class ServiceServers {
    public:
        ServiceServers() = delete;
        ServiceServers(ros::NodeHandle n, Robot* robot, std::mutex* mtx);
    private:
        Robot* robot_;
        std::mutex* mtx_lock_;

        bool inLimits(int joint_id, double joint_value) const;

        ros::ServiceServer service_absolute_;
        bool serviceAbsoluteFunction(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res);

        ros::ServiceServer service_relative_;
        bool serviceRelativeFunction(rrm_msgs::Move::Request &req, rrm_msgs::Move::Response &res);
};

#endif //CATKIN_PRVE_ZADANIE_SERVICESERVER_H
