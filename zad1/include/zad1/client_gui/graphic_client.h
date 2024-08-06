#ifndef CATKIN_PRVE_ZADANIE_GRAPHIC_CLIENT_H
#define CATKIN_PRVE_ZADANIE_GRAPHIC_CLIENT_H

#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <sys/ioctl.h>
#include <sensor_msgs/JointState.h>
#include <rrm_msgs/Move.h>
#include "terminalDefs.h"

typedef struct
{
    int i_data;
    char c_data;
    float f_data;
    char s_data[200];
} INPUT_DATA;

class GraphicClient {
    public:
        GraphicClient() = delete;
        GraphicClient(ros::NodeHandle n);
        ~GraphicClient();
    private:
        INPUT_DATA input;
        bool flg_change;
        int jointCount;
        int selectedJoint;
        std::vector<double> actualPos;
        std::vector<double> changePos;

        std::mutex data_lock;
        std::thread th_graphics;
        ros::ServiceClient moveRelativeClient;

        void g_printDivider(winsize w, int lineNum);
        void g_printCenteredTitle(winsize w, int lineNum, int titleLen, std::string menu_title);
        void g_init(bool clr_scr);
        void g_printData();
        void g_handleInput();
        void fcn_g_loop();

        ros::Subscriber jointStatesSubscriber;
        void fcn_jointStatesSubscriber(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif //CATKIN_PRVE_ZADANIE_GRAPHIC_CLIENT_H