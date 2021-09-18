#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "cmath"

class lidarReact {
    public:
        // Initialise publisher and subscriber nodes
        ros::NodeHandle n;
        ros::Publisher move_pub;
        ros::Subscriber lidar_sub; 

        // Constructor
        lidarReact(){
            move_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
            lidar_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &lidarReact::callback, this); 
        } 

        // Callback function
        void callback (const sensor_msgs::LaserScan::ConstPtr& lidarMsg) {
            int degree = 80;
            geometry_msgs::Twist move_msg;
            // See if collision within 20deg in field of view
            while (degree < 110) {
                // If object is detected less than 0.5m away from robot
                std::cout << lidarMsg->ranges[degree] << std::endl;
                if (lidarMsg->ranges[degree] < 2) {
                    move_msg.linear.x = 0; // in m/s
                    move_msg.linear.y += 1;
                    move_msg.linear.z = 0;
                    std::cout << "obstacle detected" << std::endl;
                } else {
                    move_msg.linear.x += 0.01;
                }
                degree++;
            }
            move_pub.publish(move_msg); 
        }
};

class ultraReact {
    public:
        // Initialise publisher and subscriber nodes
        ros::NodeHandle n;
        ros::Publisher move_pub;
        ros::Subscriber ultra_sub; 

        // Constructor
        ultraReact(){
            move_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
            ultra_sub = n.subscribe<sensor_msgs::PointCloud>("/RosAria/sonar", 1000, &ultraReact::callback, this); 
        } 

        void callback (const sensor_msgs::PointCloud::ConstPtr& ultraMsg) {
            geometry_msgs::Twist move_msg;
            //std::cout << "hello" << std::endl;
            double dist = std::sqrt(std::pow(ultraMsg->points[0].x,2)+std::pow(ultraMsg->points[0].y,2));
            if (dist < 0.5) {
                std::cout << "distance is " << dist << " less than 2m" << std::endl;
                move_msg.linear.x = 0; // in m/s
                move_msg.linear.y += 1;
                move_msg.linear.z = 0;
                std::cout << "obstacle detected" << std::endl;
            } else {
                std::cout << "no obstacle detected " << dist << std::endl;
                move_msg.linear.x += 0.1;
            }
            move_pub.publish(move_msg); 
        }
};
    
int main (int argc, char **argv) {
    // Initialise ROS with unique node name "reactLidar"
    ros::init(argc, argv, "reactLidar");
    lidarReact lidar_react;

    ros::spin();
    return 0;
}