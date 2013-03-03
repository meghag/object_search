
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "../include/grasp_planning.h"


#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_X 0x78
#define KEYCODE_Y 0x79
#define KEYCODE_Z 0x7a

#define KEYCODE_X_CAPS 0x58
#define KEYCODE_Y_CAPS 0x59
#define KEYCODE_Z_CAPS 0x5a

#define KEYCODE_PLUS 0x2b
#define KEYCODE_MINUS 0x2d

int kfd = 0;
struct termios cooked, raw;

void keyboardLoop()
{
    char c;
    bool dirty=false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    GraspPlanning gp;
    ros::Rate rt(1);

    int idx = 0; // which box do we edit
    //int coordinate = 0; // x y or z ? which coordinate do we affect?


    //puts("Reading from keyboard");
    //puts("---------------------------");
    //puts("Use 'WASD' to translate");
    //puts("Use 'QE' to yaw");
    //puts("Press 'Shift' to run");


    for(; ros::ok();)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        std::cout << "Keycode" << c << " 0x" << std::hex << (int)c << std::endl;

        //cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

        switch(c)
        {

        case KEYCODE_PLUS:
            idx++;
            dirty = true;
            break;

        case KEYCODE_MINUS:
            idx--;
            dirty = true;
            break;

        case KEYCODE_W:
            gp.grasps[0].bb_min[idx] += tf::Vector3(0.01,0,0);
            gp.grasps[0].bb_max[idx] += tf::Vector3(0.01,0,0);
            dirty = true;
            break;
        case KEYCODE_S:
            gp.grasps[0].bb_min[idx] -= tf::Vector3(0.01,0,0);
            gp.grasps[0].bb_max[idx] -= tf::Vector3(0.01,0,0);
            dirty = true;
            break;

        case KEYCODE_A:
            gp.grasps[0].bb_min[idx] += tf::Vector3(0,0.01,0);
            gp.grasps[0].bb_max[idx] += tf::Vector3(0,0.01,0);
            dirty = true;
            break;
        case KEYCODE_D:
            gp.grasps[0].bb_min[idx] -= tf::Vector3(0,0.01,0);
            gp.grasps[0].bb_max[idx] -= tf::Vector3(0,0.01,0);
            dirty = true;
            break;

        case KEYCODE_Q:
            gp.grasps[0].bb_min[idx] += tf::Vector3(0,0,0.01);
            gp.grasps[0].bb_max[idx] += tf::Vector3(0,0,0.01);
            dirty = true;
            break;
        case KEYCODE_E:
            gp.grasps[0].bb_min[idx] -= tf::Vector3(0,0,0.01);
            gp.grasps[0].bb_max[idx] -= tf::Vector3(0,0,0.01);
            dirty = true;
            break;



        case KEYCODE_W_CAP:
            gp.grasps[0].bb_min[idx] -= tf::Vector3(0.01,0,0);
            gp.grasps[0].bb_max[idx] += tf::Vector3(0.01,0,0);
            dirty = true;
            break;
        case KEYCODE_S_CAP:
            gp.grasps[0].bb_min[idx] += tf::Vector3(0.01,0,0);
            gp.grasps[0].bb_max[idx] -= tf::Vector3(0.01,0,0);
            dirty = true;
            break;

        case KEYCODE_A_CAP:
            gp.grasps[0].bb_min[idx] -= tf::Vector3(0,0.01,0);
            gp.grasps[0].bb_max[idx] += tf::Vector3(0,0.01,0);
            dirty = true;
            break;
        case KEYCODE_D_CAP:
            gp.grasps[0].bb_min[idx] += tf::Vector3(0,0.01,0);
            gp.grasps[0].bb_max[idx] -= tf::Vector3(0,0.01,0);
            dirty = true;
            break;

        case KEYCODE_Q_CAP:
            gp.grasps[0].bb_min[idx] -= tf::Vector3(0,0,0.01);
            gp.grasps[0].bb_max[idx] += tf::Vector3(0,0,0.01);
            dirty = true;
            break;
        case KEYCODE_E_CAP:
            gp.grasps[0].bb_min[idx] += tf::Vector3(0,0,0.01);
            gp.grasps[0].bb_max[idx] -= tf::Vector3(0,0,0.01);
            dirty = true;
            break;

        /*case KEYCODE_X:
            coordinate = 0;
            dirty = true;
            break;

        case KEYCODE_Y:
            coordinate = 1;
            dirty = true;
            break;

        case KEYCODE_Z:
            coordinate = 2;
            dirty = true;
            break;*/

        }

        if (idx < 0)
            idx = gp.grasps[0].bb_min.size() - 1;

        if (idx >= (int) gp.grasps[0].bb_min.size())
            idx = 0;

        std::cout << idx << std::endl;
        tum_os::BoxSet gbs = GraspPlanning::graspBoxSetToMsg(gp.grasps[0]);

        //std::cout << gbs << std::endl;

        if (dirty == true)
        {
            //vel_pub_.publish(cmd);
            gp.visualizeGrasp(0,"r_wrist_roll_link",idx);

            for (size_t z = 0 ; z < gp.grasps[0].bb_min.size() ; ++z)
            {
                std::cout << "act.bb_min.push_back(tf::Vector3(" << gp.grasps[0].bb_min[z].x() << "," <<
                          gp.grasps[0].bb_min[z].y() << "," <<
                          gp.grasps[0].bb_min[z].z() << "));" << std::endl;

                std::cout << "act.bb_max.push_back(tf::Vector3(" << gp.grasps[0].bb_max[z].x() << "," <<
                          gp.grasps[0].bb_max[z].y() << "," <<
                          gp.grasps[0].bb_max[z].z() << "));" << std::endl;

                std::cout << "act.bb_full.push_back(" << (gp.grasps[0].bb_full[z] ? "true" : "false") << ");" << std::endl << std::endl;
            }

        }


    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "grasp_designer");
    ros::NodeHandle n;


    keyboardLoop();

    GraspPlanning gp;
    ros::Rate rt(1);

    while (ros::ok())
    {
        gp.visualizeGrasp(0,"r_wrist_roll_link");
        rt.sleep();
        ROS_INFO("tick");
    }

    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);


}
