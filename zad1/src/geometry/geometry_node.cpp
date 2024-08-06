#include "forward_kinematics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "geometry_node");
    ros::NodeHandle n;
    ForwardKinematics forwardKinematics(n);
    ros::spin();
}