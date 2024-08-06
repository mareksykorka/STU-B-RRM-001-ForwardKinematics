#ifndef CATKIN_PRVE_ZADANIE_TOPIC_PUBLISHER_H
#define CATKIN_PRVE_ZADANIE_TOPIC_PUBLISHER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <sensor_msgs/JointState.h>
#include "robot.h"

class TopicPublisher
{
    public:
        TopicPublisher() = delete;
        TopicPublisher(ros::NodeHandle n, Robot* robot, std::mutex* mtx, std::string topicName);
        ~TopicPublisher();
    private:
        Robot* robot_;
        std::mutex* mtx_lock_;
        ros::Publisher publisher_;
        std::thread publisher_thread_;
        void publisherThreadFunction(float frequency);
};

#endif //CATKIN_PRVE_ZADANIE_TOPIC_PUBLISHER_H