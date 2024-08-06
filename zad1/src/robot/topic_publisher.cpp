#include "topic_publisher.h"

TopicPublisher::TopicPublisher(ros::NodeHandle n, Robot *robot, std::mutex *mtx, std::string topicName)
{
    this->robot_ = robot;
    this->mtx_lock_ = mtx;
    this->publisher_ = n.advertise<sensor_msgs::JointState>(topicName, 1);
    this->publisher_thread_ = std::thread(&TopicPublisher::publisherThreadFunction, this, 2);
}

TopicPublisher::~TopicPublisher()
{
    if (this->publisher_thread_.joinable())
    {
        this->publisher_thread_.join();
    }
}

void TopicPublisher::publisherThreadFunction(float frequency)
{
    ros::Rate rate(frequency);
    while (ros::ok())
    {
        sensor_msgs::JointState msg; // Create new publisher message

        this->mtx_lock_->lock(); // Lock robot access mutex
        // Fill out publisher message
        msg.header.stamp = ros::Time::now();
        msg.name.resize(this->robot_->getJointCount());
        msg.position.resize(this->robot_->getJointCount());
        for (int i = 0; i < this->robot_->getJointCount(); i++)
        {
            msg.name[i] = "joint_" + std::to_string(i + 1) ;
            msg.position[i] = this->robot_->getCurrentPosition(i);
        }
        this->mtx_lock_->unlock(); // Unlock robot access mutex

        this->publisher_.publish(msg);
        rate.sleep();
    }
}