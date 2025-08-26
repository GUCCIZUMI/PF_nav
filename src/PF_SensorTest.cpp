#include <ros/ros.h>
#include <potbot_lib/diff_drive_agent.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <potbot_lib/utility_ros.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <algorithm> 
#include <fstream>
#include <tf2/utils.h>

std::ofstream Sensor_hist_csv("/home/ros/catkin_ws/user/src/data/simulator/Sensor_hist.csv");

class MarkerDistanceAnalyzer
{
public:
    MarkerDistanceAnalyzer()
    {
        ros::NodeHandle nh;

        sub_marker_ = nh.subscribe("marker", 1000, &MarkerDistanceAnalyzer::markerCallback, this);
        sub_robot_pose_ = nh.subscribe("odom", 1000, &MarkerDistanceAnalyzer::robotPoseCallback, this);

    }

private:
    ros::Subscriber sub_marker_;
    ros::Subscriber sub_robot_pose_;
    std::vector<geometry_msgs::Pose> marker_positions_;
    std::vector<int> marker_ids_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    bool robot_pose_received_ = false;

    std::map<int, int> marker_observation_count_;
    const int Max_Observatuion = 1000;

    void markerCallback(const visualization_msgs::MarkerArray &marker_array)
    {
        marker_positions_.clear();
        marker_ids_.clear();

        for(const auto &marker : marker_array.markers)
        {
            marker_positions_.push_back(marker.pose);
            marker_ids_.push_back(marker.id);
        }

        if (robot_pose_received_)
        {
            analyzeDistance();
        }
    }

    void robotPoseCallback(const nav_msgs::Odometry &odom)
    {
        robot_x_ = odom.pose.pose.position.x;
        robot_y_ = odom.pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(odom.pose.pose.orientation);
        robot_pose_received_ = true;
    }

    void analyzeDistance()
    {
        ros::Time now = ros::Time::now();
        
        double Scan_distance_ = 0.0;
        double norm_noise_mean_Scan_distance = 0.327;
	    double norm_noise_variance_Scan_distance = 0.000856;
        double norm_noise_mean_Scan_Long_distance = 1.37; 
        double norm_noise_variance_Scan_Long_distance = 0.0138; 

        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> distribution_Scan_distance(norm_noise_mean_Scan_distance, sqrt(norm_noise_variance_Scan_distance));
        std::normal_distribution<double> distribution_Scan_Long_distance(norm_noise_mean_Scan_Long_distance, sqrt(norm_noise_variance_Scan_Long_distance));


        for(size_t i = 0; i < marker_positions_.size(); i++)
        {
            int marker_id = marker_ids_[i];

            if(marker_observation_count_[marker_id] >= Max_Observatuion)
               continue;

            double dx = marker_positions_[i].position.x - robot_x_;
            double dy = marker_positions_[i].position.y - robot_y_;
            double distance = std::sqrt(dx * dx + dy * dy);
            double angle = std::atan2(dy, dx) - robot_yaw_;
            
            if(distance <= 5.0){
            Scan_distance_ = distance + distribution_Scan_distance(generator); //マーカーとロボットの直線距離(真値の距離(近距離)：ノイズ入りセンサ値)
            }else if(5.0 < distance <= 8.0 ){
            Scan_distance_ = distance + distribution_Scan_Long_distance(generator); //マーカーとロボットの直線距離(真値の距離(遠距離)：ノイズ入りセンサ値)
            }else{
            Scan_distance_ = 0; //マーカーとロボットの直線距離(真値の距離(それ以外)：ノイズ入りセンサ値)
            }
            
            ROS_INFO("MarkerID: %d, Distance: %.3f m (Count: %d) ", marker_ids_[i], Scan_distance_, marker_observation_count_[marker_id]+1);
            Sensor_hist_csv <<  now.toSec() << "," << marker_id << "," << Scan_distance_ <<  std::endl;
            
            marker_observation_count_[marker_id]++;

            if(checkAllMarkersComplete())
            {
                ROS_INFO("All markers have reached %d observations. Shutting down...", Max_Observatuion);
                ros::shutdown();
                return;
            }
        }
    }
    bool checkAllMarkersComplete()
    {
        if (marker_observation_count_.empty())
            return false;

        for (const auto &entry : marker_observation_count_)
        {
            if (entry.second < Max_Observatuion)
                return false;
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_distance_analyzer");
    MarkerDistanceAnalyzer analyzer;
    ros::spin();
    return 0;
}