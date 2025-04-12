#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>//ロボットの指令値(速度)用ヘッダー
#include <nav_msgs/Odometry.h>

class PoseAlignment
{
private:
    ros::Publisher pub_cmd_;
    ros::Subscriber sub_odom_;

    int robot_action_ = 0;
    
    double liner_X_ = 0;
    double max_liner_velocity_ = 0;
    double target_X_ = 0;
    double target_y_ = 0;
    double target_yaw_ = 0;
    double rotation_gain_p_ = 1;
    double foward_gain_p_ = 1;
    double yaw_now = 0;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
public:
    PoseAlignment(/* args */);
    ~PoseAlignment();
};

PoseAlignment::PoseAlignment(/* args */)
{
    ros::NodeHandle n("~");

    n.getParam("liner_X", liner_X_);
    n.getParam("max_liner_velocity", max_liner_velocity_);
    n.getParam("target_X", target_X_);
    n.getParam("target_y", target_y_);
	n.getParam("target_yaw", target_yaw_);
    n.getParam("rotation_gain_p", rotation_gain_p_);
    n.getParam("foward_gain_p", foward_gain_p_);
    ros::NodeHandle nhSub;//ノードハンドル

    sub_odom_ = nhSub.subscribe("/robot1/odom", 1000, &PoseAlignment::odomCallback,this);

    ros::NodeHandle nh;
    pub_cmd_ = nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);
}

PoseAlignment::~PoseAlignment()
{
}


//コールバック関数
void PoseAlignment::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const geometry_msgs::Pose robot_pose = msg->pose.pose;
    // ROS_INFO_STREAM(robot_pose.position.x << " : " << robot_pose.position.y << " : " << tf2::getYaw(robot_pose.orientation));

    
    double error_yaw = target_yaw_ - tf2::getYaw(robot_pose.orientation);
    geometry_msgs::Twist cmd_vel;
    if(robot_action_ == 1)
    {
        double x_now;
        double X_now;
        double error_X = target_X_ - x_now;
        double v1 = error_X* foward_gain_p_;
        if(v1 > max_liner_velocity_)
        {
            v1 = max_liner_velocity_;
        }
        cmd_vel.linear.x = v1;
        cmd_vel.angular.z = 0;
        if(X_now = target_X_ )
        {
            robot_action_ = 2;
        }

    } 
    else if(robot_action_ == 2)
    {
        
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = rotation_gain_p_*error_yaw;
        pub_cmd_.publish(cmd_vel);

        if( yaw_now = target_yaw_)
        {
            robot_action_ = 3;
        }

    }
    else if(robot_action_ == 3)
    {
        double y_now;
        double error_y = target_y_ - y_now;
        double v2 = error_y* foward_gain_p_;
        if(v2 > max_liner_velocity_)
        {
            v2 = max_liner_velocity_;
        }
        cmd_vel.linear.x = v2;
        cmd_vel.angular.z = 0;

        if(y_now = target_y_)
        {
            robot_action_ = 0;
        }
    }
    else if(robot_action_ == 0)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }
    
    
    
}

//メイン関数
int main(int argc,char **argv){
	ros::init(argc,argv,"kobuki_pub_cmd");//rosを初期化
	PoseAlignment pa;

	ros::spin();//トピック更新待機
			
	return 0;
}