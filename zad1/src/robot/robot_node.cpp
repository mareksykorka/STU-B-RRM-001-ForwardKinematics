#include <ros/ros.h>
#include <mutex>
#include "robot.h"
#include "topic_publisher.h"
#include "service_servers.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle n;

    std::mutex mtx_lock;

    Robot robot = Robot(n);
    TopicPublisher jointStatesPublisher(n,&robot,&mtx_lock,"/joint_states");
    ServiceServers serviceServers(n, &robot, &mtx_lock);
    ros::spin();
}