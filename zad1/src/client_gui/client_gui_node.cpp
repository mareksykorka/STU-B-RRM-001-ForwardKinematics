#include <ros/ros.h>
#include "graphic_client.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client_gui_node");
    ros::NodeHandle n;
    GraphicClient graphicClient(n);
    ros::spin();
}