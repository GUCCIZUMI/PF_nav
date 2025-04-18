#include <ros/ros.h>
#include <potbot_lib/utility_ros.h>
#include <potbot_lib/pid.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <random>


geometry_msgs::Twist keyboard_linear;
geometry_msgs::Twist robot_velocity;

void keyboard_callback(const geometry_msgs::Twist& keyboard_msgs)
{
    keyboard_linear = keyboard_msgs;
    robot_velocity.linear.x = keyboard_linear.linear.x;
    robot_velocity.linear.y = keyboard_linear.linear.y;

    std::cout << "テスト速度 X: " << robot_velocity.linear.x 
              << " Y: " << robot_velocity.linear.y << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "PF_keyboard");
    ros::NodeHandle nh;

    ros::Publisher  keyboard_pub_twist;
    
    ros::Subscriber sub_keyboard = nh.subscribe("/cmd_vel", 1, keyboard_callback);

    ros::spin();  // コールバックを処理
    return 0;
}