#include "graphic_client.h"

GraphicClient::GraphicClient(ros::NodeHandle t_n)
{
    // Initialize Variables
    flg_change = false;
    jointCount = 0;
    selectedJoint = 0;
    // Create all functional instances
    moveRelativeClient = t_n.serviceClient<rrm_msgs::Move>("/move_relative");
    jointStatesSubscriber = t_n.subscribe("/joint_states", 1, &GraphicClient::fcn_jointStatesSubscriber, this);
    th_graphics = std::thread(&GraphicClient::fcn_g_loop, this);
}
GraphicClient::~GraphicClient()
{
    if (th_graphics.joinable())
    {
        th_graphics.join();
    }
}

void GraphicClient::fcn_jointStatesSubscriber(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Check jointCount published by robot itself
    data_lock.lock();
    if(jointCount != msg->position.size())
    {
        jointCount = msg->position.size();
        actualPos.resize(jointCount);
        changePos.resize(jointCount);
    }

    // Check positions_ that are displayed and real
    bool change = false;
    for (int i = 0; i < jointCount; i++)
    {
        if(actualPos[i] != msg->position[i])
        {
            change = true;
            break;
        }
    }
    data_lock.unlock();

    // If they are diffrent load new positions_
    if(change)
    {
        actualPos = msg->position;
        flg_change = true;
        g_printData();
    }
}
void GraphicClient::g_printDivider(winsize w, int lineNum)
{
    set_cursor(0,lineNum);
    for (int i=0;i<w.ws_col;i++)
        printf("#");
}
void GraphicClient::g_printCenteredTitle(winsize w, int lineNum, int titleLen, std::string menu_title)
{
    set_cursor(((w.ws_col/2) - (titleLen/2)),lineNum);
    printf("%s", menu_title.c_str());
}
void GraphicClient::g_init(bool clr_scr)
{
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

    std::string menu_title = " Robot joint control - client ";
    int l_menu = menu_title.size();

    std::string question_title = " Controls ";
    int l_question = question_title.size();

    std::string input_title = " Input Box ";
    int l_input = input_title.size();

    std::string debug_title = " Additional Info ";
    int l_debug = debug_title.size();

    if(clr_scr)
        cls();

    g_printDivider(w, TERM_POS_MENU_S);
    g_printCenteredTitle(w, TERM_POS_MENU_S, l_menu, menu_title);

    set_cursor(0,TERM_POS_TABL_S);
    //      |   | Joint # |  0.0000000 |  0.0000000 |  0.0000000 |
    printf("| X | Joint # |  Robot Pos |   New Pos  |   Change   |");

    g_printDivider(w, TERM_POS_CTRL_S + jointCount);
    g_printCenteredTitle(w, TERM_POS_CTRL_S + jointCount, l_question, question_title);

    set_cursor(0,TERM_POS_CTRL + jointCount);
    printf(" [w] - Move cursor UP\n [s] - Move cursor DOWN\n [m] - Apply changes to the robot \n [e] - Exit this client");

    g_printDivider(w, TERM_POS_INPT_S + jointCount);
    g_printCenteredTitle(w, TERM_POS_INPT_S + jointCount, l_input, input_title);

    g_printDivider(w, TERM_POS_STAT_S + jointCount);
    g_printCenteredTitle(w, TERM_POS_STAT_S + jointCount, l_debug, debug_title);
    g_printDivider(w, TERM_POS_STAT_E + jointCount);

    set_cursor(0,TERM_POS_INPT);

    fflush(stdout);
}
void GraphicClient::g_printData()
{
    data_lock.lock();
    set_cursor(0,TERM_POS_TABL_S+1);
    for (int i = 0; i < jointCount; i++)
    {
        if(selectedJoint == i)
            printf("|-->|");
        else
            printf("|   |");

        printf(" %d_joint |", (i+1));

        if(actualPos[i] >= 0)
            printf("  %.7f |", actualPos[i]);
        else
            printf(" %.7f |", actualPos[i]);

        if((actualPos[i]+changePos[i]) >= 0)
            printf("  %.7f |", actualPos[i]+changePos[i]);
        else
            printf(" %.7f |", actualPos[i]+changePos[i]);

        if(changePos[i] >= 0)
            printf("  %.7f |", changePos[i]);
        else
            printf(" %.7f |", changePos[i]);

        printf("\n");
    }
    set_cursor(0,TERM_POS_INPT + jointCount);
    data_lock.unlock();
    fflush(stdout);
}
void GraphicClient::g_handleInput()
{
    data_lock.lock();
    switch(input.c_data)
    {
        case 'w':
        {
            if(selectedJoint-1 >= 0)
            {
                selectedJoint--;
                set_cursor(0,TERM_POS_STAT + jointCount);
                clrline();
                fflush(stdout);
                flg_change = true;
            }
            else
            {
                for (int i = TERM_POS_STAT + jointCount; i < TERM_POS_STAT_E + jointCount; i++) {
                    set_cursor(0,i);
                    clrline();
                }
                set_cursor(0,TERM_POS_STAT + jointCount);
                printf("Can't go any higher than that ¯\\_(ツ)_/¯.");
                fflush(stdout);
            }
            break;
        }
        case 's':
        {
            if(selectedJoint+1 < jointCount)
            {
                selectedJoint++;
                set_cursor(0,TERM_POS_STAT + jointCount);
                clrline();
                fflush(stdout);
                flg_change = true;
            }
            else
            {
                for (int i = TERM_POS_STAT + jointCount; i < TERM_POS_STAT_E + jointCount; i++) {
                    set_cursor(0,i);
                    clrline();
                }
                set_cursor(0,TERM_POS_STAT + jointCount);
                printf("Can't go any lower than that ¯\\_(ツ)_/¯.");
                fflush(stdout);
            }
            break;
        }
        case 'm':
        {
            rrm_msgs::Move msg;
            msg.request.positions.resize(jointCount);
            for (int i = 0; i < jointCount; i++)
            {
                msg.request.positions[i] = changePos[i];
            }
            if(moveRelativeClient.call(msg))
            {
                for (int i = TERM_POS_STAT + jointCount; i < TERM_POS_STAT_E + jointCount; i++) {
                    set_cursor(0,i);
                    clrline();
                }
                set_cursor(0,TERM_POS_STAT + jointCount);
                printf("RESPONSE is: %s", msg.response.message.c_str());
                flg_change = true;
            }
            else
            {
                set_cursor(0,TERM_POS_STAT + jointCount);
                clrline();
                printf("RESPONSE is: %s", msg.response.message.c_str());
            }
            break;
        }
        case 'e':
        {
            cls();
            fflush(stdout);
            exit(1);
        }
        default:
        {
            for (int i = TERM_POS_STAT + jointCount; i < TERM_POS_STAT_E + jointCount; i++) {
                set_cursor(0,i);
                clrline();
            }
            set_cursor(0,TERM_POS_STAT + jointCount);
            fflush(stdout);
            changePos[selectedJoint] = input.f_data;
            flg_change = true;
            break;
        }
    }
    data_lock.unlock();
}
void GraphicClient::fcn_g_loop()
{
    g_init(1);
    set_cursor(0, TERM_POS_STAT);
    printf("Waiting for joint count from topic /joint_states");
    set_cursor(0,TERM_POS_INPT + jointCount);
    fflush(stdout);
    while(jointCount == 0)
        sleep(1);

    g_init(1);
    g_printData();
    while(true)
    {
        if(flg_change)
        {
            g_printData();
            flg_change = false;
        }
        set_cursor(0,TERM_POS_INPT + jointCount);
        clrline();
        fflush(stdout);
        //Awaiting input
        memset(&input, 0, sizeof(input));
        fgets(input.s_data, sizeof(input.s_data), stdin);
        //Try parse input into diffrent data types
        input.c_data = input.s_data[0];
        input.i_data = atoi(input.s_data);
        input.f_data = atof(input.s_data);
        if(input.c_data == '\n')
            input.c_data = '-';
        g_handleInput();
    }
}