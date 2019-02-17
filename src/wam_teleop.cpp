/* 
 * lpetrich 13/02/19
 */

#include "teleop/wam_teleop.h"

WamTeleop::WamTeleop() {
    ros::NodeHandle nh("~");
    publish_realtime = true;
    velocity_gains = false;
    quit = false;
    new_k = false;
    XY = true;
    YZ = false;
    XZ = false;
    dof = 0;
    k = '\0';
    set_direction_vector(0.0, 0.0, 0.0);
    arm = new ArmControl(nh);
    do {
        ROS_INFO_STREAM("Control Center: Waiting to find robot DOF " << dof);
        ros::Duration(1.0).sleep();
        dof = arm->get_dof();
    } while (dof == 0);
    if (dof == 7) {
        bhand = new BHandControl("/zeus", nh);
        // initialize_hand();
    } 
    arm->lock_joint_position(false);
    ROS_INFO_STREAM("Robot has " << dof << " DOF");
    rt_cart_vel_pub = nh.advertise<wam_msgs::RTVelocity>("/zeus/wam/cartesian_velocity_control", 100);
    std::string topic = "/user_interface/keyboard";
    keyboard_sub = nh.subscribe(topic, 1, &WamTeleop::keyboard_cb, this);
    ROS_INFO_STREAM("WamTeleop now subscribing to " << topic);
    std::cout << "***************************************************************************************\n"
              << "Use the arrow keys to move in the XY-plane, press spacebar to toggle between planes" << std::endl;

}

WamTeleop::~WamTeleop() 
{
    delete arm;
    delete bhand;
    keyboard_sub.shutdown();
    rt_cart_vel_pub.shutdown();
}

void WamTeleop::initialize_hand()
{
    if (dof == 7) {
        ROS_INFO_STREAM("Initializing, please wait...");
        while (!arm->move_to_initial_position()) { continue; }
        ros::Duration(4).sleep();            
        while (!bhand->initialize()) { continue; }
        while (!bhand->close_spread()) { continue; }
        ros::Duration(10).sleep();
        ROS_INFO_STREAM("Initialization complete");
    }
}

void WamTeleop::set_direction_vector(double x, double y, double z) {
    direction_vector[0] = x;
    direction_vector[1] = y;
    direction_vector[2] = z;
}

void WamTeleop::loop() {
    ros::Rate r(PUBLISH_FREQ);
    while (ros::ok() && !quit) {
        // std::cout << "Command: " << k << std::endl;
        if (new_k) {
            if ((k == KEYCODE_UP_ARROW) && (XY || XZ)) 
                set_direction_vector(1.0, 0.0, 0.0);
            else if ((k == KEYCODE_DOWN_ARROW) && (XY || XZ))
                set_direction_vector(-1.0, 0.0, 0.0);
            else if ((k == KEYCODE_RIGHT_ARROW) && (YZ || XZ))
                set_direction_vector(0.0, 0.0, 1.0);
            else if ((k == KEYCODE_LEFT_ARROW) && (YZ || XZ))
                set_direction_vector(0.0, 0.0, -1.0);
            else if ((k == KEYCODE_UP_ARROW && YZ) || (k == KEYCODE_RIGHT_ARROW && XY))
                set_direction_vector(0.0, 1.0, 0.0);
            else if ((k == KEYCODE_DOWN_ARROW && YZ) || (k == KEYCODE_LEFT_ARROW && XY))
                set_direction_vector(0.0, -1.0, 0.0);
            else
                set_direction_vector(0.0, 0.0, 0.0);

            new_k = false;

        }
        ros::spinOnce();
        r.sleep();
    }
}

void WamTeleop::publish_loop() {
    ros::Rate pub_rate(PUBLISH_FREQ);
    while (ros::ok()) {
        try {
            if (publish_realtime) {
                wam_msgs::RTVelocity cart_vel_cmd;
                cart_vel_cmd.v_magnitude = 0.1;
                for (int i = 0; i < 3; ++i) {
                    cart_vel_cmd.v_direction[i] = direction_vector[i];
                }
                cart_vel_cmd.kp = 0.0;
                cart_vel_cmd.change_gains = false;
                if (velocity_gains) {
                    velocity_gains = false;
                }
                rt_cart_vel_pub.publish(cart_vel_cmd);
            }
        } catch (...) {
            ROS_WARN_STREAM("Exception Occured");
        }
        ros::spinOnce();
        pub_rate.sleep();
    }
}

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "wam_teleop");
    // don't remove spinner lines or arm won't subscribe to wam topics
    ros::AsyncSpinner spinner(0);
    spinner.start();
    WamTeleop WT;
    boost::thread publish_thread(&WamTeleop::publish_loop, &WT);
    try {
        WT.loop();
    } catch (...) {
        ROS_WARN_STREAM("Exception Occured.");
    }
    return 0;
}
