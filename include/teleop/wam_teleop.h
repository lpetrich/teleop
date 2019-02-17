#ifndef WAM_TELEOP_H
#define WAM_TELEOP_H

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <exception>
#include <boost/thread.hpp> 
#include <dynamic_reconfigure/server.h>
#include "wam_control/arm_control.h"
#include "wam_control/bhand_control.h"
#include "wam_control/misc_utilities.h"
#include "wam_msgs/RTOrtn.h"
#include "wam_msgs/RTVelocity.h"
#include "std_msgs/Char.h"
#include "wam_srvs/Connect.h"
#include "user_interface/keycodes.h"

static const int PUBLISH_FREQ = 60; // Default Control Loop / Publishing Frequency

class WamTeleop 
{
    ArmControl *arm;
    BHandControl *bhand;
    char k;
    bool publish_realtime;
    bool velocity_gains;
    bool new_k;
    bool quit;
    bool XY;
    bool YZ;
    bool XZ;
    int dof;
    Eigen::Vector3d direction_vector;

    public:
        WamTeleop();
        ~WamTeleop();
        void initialize_hand();
        void set_direction_vector(double x, double y, double z);
        void loop();
        void publish_loop();

    private:
        ros::Publisher rt_cart_vel_pub;
        ros::Subscriber keyboard_sub;
        void keyboard_cb(std_msgs::Char msg) {
            char c = msg.data;
            if (c == KEYCODE_QUIT) {
                std::cout << "shutdown command received" << std::endl;
                quit = true;
            } 
            else if (c == KEYCODE_SPACE) {
                // current_plane = (current_plane + 1) % 3;
                std::cout << "Mode switch to ";
                if (XY) {
                    YZ = true;
                    XY = XZ = false;
                    std::cout << "XY-plane" << std::endl;
                } else if (YZ) {
                    XZ = true;
                    XY = YZ = false;
                    std::cout << "YZ-plane" << std::endl;
                } else {
                    XY = true;
                    YZ = XZ = false;
                    std::cout << "XZ-plane" << std::endl;
                }
            } else if (!new_k) {
                k = c;
                new_k = true;
            }
        }
};

#endif // WAM_TELEOP_H