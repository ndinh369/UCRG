#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" // Motor Commands
#include "sensor_msgs/Joy.h" // Joystick
//#include "sensor_msgs/LaserScan.h" // Laser Data
#include <std_srvs/Empty.h>

/*// Define the robot direction of movement
typedef enum _ROBOT_MOVEMENT {
    STOP = 0,
    FAST_FORWARD,
    MED_FORWARD,
} ROBOT_MOVEMENT;*/

class CtrlMove {
    public:
        // Initialise publisher and subscriber nodes
        ros::NodeHandle n;
        ros::Publisher move_pub;
        geometry_msgs::Twist move_msg;
        ros::Subscriber joy_sub; 
        //ros::Subscriber laser_sub;
        //sensor_msgs::LaserScan laser_msg;
        ros::ServiceClient motor_on;
        ros::ServiceClient motor_off;
        //int override = 0;
        float prev_straight = 0;

        // Constructor
        CtrlMove(){
            move_pub = n.advertise<geometry_msgs::Twist>(/*"/RosAria/cmd_vel"*/"/cmd_vel", 1000);
            joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1000, &CtrlMove::joy_callback, this); 
            //laser_sub =  n.subscribe<sensor_msgs::LaserScan>("/RosAria/scan", 1000, &CtrlMove::laser_callback, this);
            motor_on = n.serviceClient<std_srvs::Empty>("/RosAria/enable_motors");
            motor_off = n.serviceClient<std_srvs::Empty>("/RosAria/disable_motors");
        } 
/*
        bool robot_move(const ROBOT_MOVEMENT move_type)
        {
            if (move_type == STOP) {
                ROS_INFO("[ROBOT] STOP OVERRIDE, obstacle is very close! \n");

                move_msg.angular.z = 0.0;
                move_msg.linear.x = 0.0;
            }

            else if (move_type == FAST_FORWARD) {
                ROS_INFO("[ROBOT] Fast FORWARD, no obstacles! \n");
                move_msg.angular.z = 0.0;
                move_msg.linear.x = 0.25;
            }

            else if (move_type == MED_FORWARD) {
                ROS_INFO("[ROBOT] Medium FORWARD, obstacle is getting closer! \n");
                move_msg.angular.z = 0.0;
                move_msg.linear.x = 0.1;
            }

            else {
                ROS_INFO("[ROBOT_MOVE] Move type wrong! \n");
                return false;
            }

            //Publish motor commands to the robot and wait 10ms
            move_pub.publish(move_msg);
            usleep(10);
            return true;
        }
*/
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

            if (/*R_trigg_val > -L_trigg_val && */joyMsg->buttons[5] == 0 /*&& override != 2*/) {
                //if (override == 0) {
                    if( R_trigg_val+L_trigg_val <0){
                        move_msg.linear.x =0;
                    } else {
                        move_msg.linear.x = R_trigg_val+L_trigg_val;
                    }
                /*} else if (override == 1) {
                    move_msg.linear.x = slow;
                    R_trigg_val = move_msg.linear.x;
                } */
                // Orientate left and right
                move_msg.angular.z = joyMsg->axes[0];
            } else if (/*R_trigg_val > -L_trigg_val &&*/ joyMsg->buttons[5] == 1) {
                move_msg.linear.x = -R_trigg_val;
                move_msg.angular.z = joyMsg->axes[0];
            } /*else if (R_trigg_val < -L_trigg_val ) {
                move_msg.linear.x = L_trigg_val;
                // Orientate left and right
                move_msg.angular.z = joyMsg->axes[0];
            }*/ else {
                move_msg.linear.x = 0;
                R_trigg_val = move_msg.linear.x;
                move_msg.angular.z = 0;
            }
            std::cout << "R trigger " << R_trigg_val << std::endl;
            std::cout << "L trigger " << L_trigg_val << std::endl;
            std::cout << "L joystick " << /*joyMsg->axes[0]*/ move_msg.angular.z << std::endl;
            move_pub.publish(move_msg);  
            
            //override = 0;
        }
/*
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
            // Read and process laser scan values
            laser_msg = *scan_msg;
            std::vector<float> laser_ranges;
            laser_ranges = laser_msg.ranges;
            size_t range_size = 360;
            float left_side = 0.0, right_side = 0.0, curr_straight = 0.0, f_left =0.0, f_right=0.0;
            float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
            for (size_t i = 0; i < range_size; i++) {
                if ((i > 267 && i < 273)) {
                    right_side += laser_ranges[i];
                } else if ((i > 312 && i < 318)) {
                    f_left += laser_ranges[i];
                } else if ((i >= 0 && i < 3) || (i > 358)) {
                    curr_straight += laser_ranges[i];
                } else if ((i > 42 && i < 48)) {
                    f_right += laser_ranges[i];
                } else if ((i > 87 && i < 93)){
                    left_side += laser_ranges[i];
                }
            }

            float avg = 5;
            left_side = left_side/avg;
            f_left = f_left/avg;
            curr_straight = curr_straight/avg;
            f_right = f_right/avg;
            right_side = right_side/avg;

            float min_dist = 0.3;
            float straight_change;

            straight_change = curr_straight - prev_straight;

            if (curr_straight < min_dist) {
            override = 2;
            std::cout << "override stop: " << override << std::endl;
            
            robot_move(STOP);
            } else {
                if (straight_change < 0 && curr_straight < min_dist + 0.2) {
                    //robot_move(MED_FORWARD);
                    override = 1;
                } else {
                    //robot_move(FAST_FORWARD);
                }
            }

            prev_straight = curr_straight;    
        }*/
};
int main (int argc, char **argv) {
    // Initialise ROS with unique node name "moveCtrl"
    ros::init(argc, argv, "moveCtrl");
    CtrlMove control_move;
    ros::spin();
    return 0;
}
