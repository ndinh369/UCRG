#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" // Motor Commands
#include "sensor_msgs/Joy.h" // Joystick
#include <std_srvs/Empty.h>

class CtrlMove {
    public:
        // Initialise publisher and subscriber nodes
        ros::NodeHandle n;
        ros::Publisher move_pub;
        geometry_msgs::Twist move_msg;
        ros::Subscriber joy_sub; 
        ros::ServiceClient motor_on;
        ros::ServiceClient motor_off;
        float prev_straight = 0;

        // Constructor with ROS publishers and subscribers
        CtrlMove(){
            move_pub = n.advertise<geometry_msgs::Twist>(/*"/RosAria/cmd_vel"*/"/cmd_vel", 1000);
            joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1000, &CtrlMove::joy_callback, this); 
            motor_on = n.serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
            motor_off = n.serviceClient<std_srvs::Empty>("/RosAria/disable_motors");
        } 

        // Joy callback function
        void joy_callback (const sensor_msgs::Joy::ConstPtr& joyMsg) {
            std_srvs::Empty motor;
            double R_trigg_val, L_trigg_val;
            //std::cout << "override: " << override << std::endl;

            // Turn on robot (A button)
            if (joyMsg->buttons[0] == 1){
                if (motor_on.call(motor)) {
                    std::cout << "Turn on robot" <<  std::endl;
                } else {
                    std::cout << "Failed to turn on" << std::endl;
                }
            }

            // Turn off robot (B button)
            if (joyMsg->buttons[1] == 1){
                if (motor_off.call(motor)) {
                    std::cout << "Turn off robot" <<  std::endl;
                } else {
                    std::cout << "Failed to turn off" << std::endl;
                }
            }

            // Accelerate (RT)
            double max = 0.3;
            double slow = 0.1;
            R_trigg_val = (-0.5*joyMsg->axes[5]+0.5)*max;
            L_trigg_val = -(-0.5*joyMsg->axes[2]+0.5)*max;

            if (joyMsg->buttons[5] == 0) {
                //if (override == 0) {
                    if( R_trigg_val+L_trigg_val <0){
                        move_msg.linear.x =0;
                    } else {
                        move_msg.linear.x = R_trigg_val+L_trigg_val;
                    }
                // Orientate left and right
                move_msg.angular.z = joyMsg->axes[0];
            } else if (joyMsg->buttons[5] == 1) {
                move_msg.linear.x = -R_trigg_val;
                move_msg.angular.z = joyMsg->axes[0];
            } else {
                move_msg.linear.x = 0;
                R_trigg_val = move_msg.linear.x;
                move_msg.angular.z = 0;
            }
            std::cout << "R trigger " << R_trigg_val << std::endl;
            std::cout << "L trigger " << L_trigg_val << std::endl;
            std::cout << "L joystick " << move_msg.angular.z << std::endl;
            move_pub.publish(move_msg);  
        }
};

int main (int argc, char **argv) {
    // Initialise ROS with unique node name "moveCtrl"
    ros::init(argc, argv, "moveCtrl");
    CtrlMove control_move;
    ros::spin();
    return 0;
}
