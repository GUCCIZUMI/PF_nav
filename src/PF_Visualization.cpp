#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <potbot_lib/Utility.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <algorithm> 
#include <fstream>

//構造体の定義(本来はincludeを作成してやるべき)
struct ParticlePosition {
    double x;
    double y;
    double yaw;
};

class PFVisualization
{
private:
    ros::Subscriber sub_marker_, sub_robot_pose_, sub_particle_pose_;
    ros::Publisher pub_estimated_robot_;

    nav_msgs::Odometry odom_msg_;

    //各記録用(ofstream定義)
    // std::ofstream record_end_angle_; //観測用カメラの測定最終角度
    // std::ofstream record_robot_pose_yaw; //ロボットの姿勢角度(観測では上記の2つの真ん中になっていれば良い)
    // std::ofstream record_angle; //ロボットとマーカーの角度(atan関数が機能しているかの確認)
    // std::ofstream particle_position; //各パーティクルの位置(動作モデル後)
    // std::ofstream marker_particle_information; //パーティクルとマーカー間の距離と角度(観測モデル使用情報)
    // std::ofstream particle_weight;//各パーティクルの尤度(距離、角度、尤度、合計)
    // std::ofstream particle_Norm_weight;//各パーティクルの正規化後の尤度(正規化前尤度、正規化前合計尤度、正規化前尤度、正規化前合計尤度(1になる))

    std::vector<ParticlePosition> particle_positions_; //パーティクル位置定義(動作モデル後)
    std::vector<geometry_msgs::Pose> marker_positions_; //マーカー位置定義
    std::vector<int> marker_ids_;; //マーカーid
    std::vector<double> Likelihood_; //各マーカの重み
    std::vector<double> Norm_Likelihood_;
    //std::vector<double> particle_position_x; //各パーティクルのx座標
    //std::vector<double> particle_position_y; //各パーティクルのx座標
    //std::vector<double> particle_position_yaw; //各パーティクルのx座標

    double robot_pose_x_ = 0.0, robot_pose_y_ = 0.0, robot_pose_z_ = 0.0, robot_pose_yaw_ = 0.0; //ロボット位置の真値(定義)
    double Robot_distance_ = 0.0, Robot_angle_ = 0.0; //ロボットとマーカーの距離と角度(角度に関してはロボット座標系における角度に変更する必要あるかも)
    double particle_position_x_ = 0.0, particle_position_y_ = 0.0, Particle_Est_RobotYaw_ = 0.0; //パーティクルの位置(使ってない)
    double dis_X_ = 0.0, dis_Y_ = 0.0; //各パーティクルの各マーカー間のX軸、Y軸誤差
    double radius_ = 3.0, start_angle_ = 0.0, end_angle_ = 0.0, angle_area_ = 85.2 * M_PI / 180; //ロボットの観測範囲の定義
    double dis_var_ = 0.000856, ang_var_ = 0.000464;
    double total_weight_ = 0.0;
    bool sebscribed_robot_pose_ = false;
    bool sebscribed_landmark_pose_ = false;
    bool sebscribed_particle_pose_ = false;
public:
    PFVisualization(/* args */);
    ~PFVisualization(){};

    void markerCallback(const visualization_msgs::MarkerArray& marker_array);
    void robotPoseCallback(const nav_msgs::Odometry& odom_pose);
    void particleCallback(const geometry_msgs::PoseArray& particle_pose);
    void initLiklihood();
    void normLiklihood();
    void getObservedLandmark(std::vector<int>& in_range);
    void getLikelihood(size_t marker_id);
    void getEstimatedRobotPose();
    void localization();
};

PFVisualization::PFVisualization(/* args */)
{
    // ros::NodeHandle n("~");
	// n.getParam("particle_num", control_frequency); //launchファイルから取得周期(50.0)

    // record_end_angle_ = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/record_end_angle__txt");
    // record_robot_pose_yaw = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/record_robot_pose_yaw_txt");
    // record_angle = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/record_angle_txt");
    // particle_position = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/particle_position_txt");
    // marker_particle_information = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/ marker_particle_information_txt");
    // particle_weight = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/ particle_weight_txt");
    // particle_Norm_weight = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/ particle_Norm_weight_txt");

    ros::NodeHandle nh;
    ros::Rate rate(50.0);

    // サブスクライバの作成
    sub_marker_ = nh.subscribe("marker", 1000, &PFVisualization::markerCallback,this);
    sub_robot_pose_ = nh.subscribe("odom", 1000, &PFVisualization::robotPoseCallback,this);
    sub_particle_pose_ = nh.subscribe("particles", 1000, &PFVisualization::particleCallback,this);

    pub_estimated_robot_ = nh.advertise<nav_msgs::Odometry>("odom/estimated", 1);
}

void PFVisualization::markerCallback(const visualization_msgs::MarkerArray& marker_array)
{   
    sebscribed_landmark_pose_ = true;
    
    marker_positions_.clear(); //マーカの
    marker_ids_.clear();

	for(const auto& marker : marker_array.markers)
    {
        geometry_msgs::Pose marker_pose = marker.pose;
        
        marker_ids_.push_back(marker.id);
        marker_positions_.push_back(marker_pose);

        // ROS_INFO("Marker ID: %d, Position: [x: %f, y: %f, z: %f]", 
        //           marker.id, marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
    }
}

//ロボットのコールバック関数
void PFVisualization::robotPoseCallback(const nav_msgs::Odometry& odom_pose)
{
    sebscribed_robot_pose_ = true;
    odom_msg_ = odom_pose;
    robot_pose_x_ = odom_pose.pose.pose.position.x;
    robot_pose_y_ = odom_pose.pose.pose.position.y;
    robot_pose_z_ = odom_pose.pose.pose.position.z;

    robot_pose_yaw_ = potbot_lib::utility::get_Yaw(odom_pose.pose.pose.orientation);
    
    if (sebscribed_landmark_pose_ && sebscribed_robot_pose_ && sebscribed_particle_pose_)
    {
        localization();
    }    
    
}

//パーティクルのコールバック関数
void PFVisualization::particleCallback(const geometry_msgs::PoseArray& particle_pose)
{
    sebscribed_particle_pose_ = true;

    particle_positions_.clear();
    //particle_position.clear()
    for(const auto& particle_pose : particle_pose.poses)
    {
        ParticlePosition particle_position;
        particle_position.x = particle_pose.position.x;
        particle_position.y = particle_pose.position.y;
        particle_position.yaw = potbot_lib::utility::get_Yaw(particle_pose.orientation);
        // ROS_INFO(" Position: [x: %f, y: %f, z: %f]", 
        //            particle_pose.position.x, particle_pose.position.y, particle_pose.position.z);
        
        particle_positions_.push_back(particle_position);

        //particle_position << "particle_position.x" << particle_pose.position.x << "particle_position.y" << particle_pose.position.y << "particle_position.yaw" << particle_position_yaw << std::endl;
    }
  
}

void PFVisualization::localization()
{
    std::vector<int> in_range_ids;
    getObservedLandmark(in_range_ids);
    initLiklihood();

    for (const auto& marker_id:in_range_ids)
    {
        getLikelihood(marker_id);
    }
    normLiklihood();

    getEstimatedRobotPose();
}

void PFVisualization::initLiklihood()
{
    Likelihood_.clear();
    Likelihood_.resize(particle_positions_.size());
    std::fill(Likelihood_.begin(), Likelihood_.end(), 1.0);
}

void PFVisualization::normLiklihood()
{
    double sum = std::accumulate(Likelihood_.begin(), Likelihood_.end(), 0.0);
    for(auto& val:Likelihood_) val/=sum;
}

//範囲内マーカー観測プロセス
void PFVisualization::getObservedLandmark(std::vector<int>& in_range)
{
    in_range.clear();
    for(size_t i = 0; i < marker_positions_.size(); ++i)
    {
        const auto& marker_pose = marker_positions_[i];
        double dx = marker_pose.position.x - robot_pose_x_;
        double dy = marker_pose.position.y - robot_pose_y_;
        Robot_distance_ = std::sqrt(dx * dx + dy * dy);
        
        if(Robot_distance_ > radius_){
            continue;
        }

        Robot_angle_ = robot_pose_yaw_ - std::atan2(dy, dx);
        // if (Robot_angle_ < 0){
        //     Robot_angle_ += 2 * M_PI;
        // }
        
        // record_angle << Robot_angle_  << std :: endl;

        // double start_angle_ = robot_pose_yaw_ - angle_area_ / 2;
        // double end_angle_ = robot_pose_yaw_ + angle_area_ / 2;
        
        // record_start_angle_ << start_angle_  << std :: endl;
        // record_end_angle_ << end_angle_  << std :: endl;
        // record_robot_pose_yaw << robot_pose_yaw   << std :: endl;

        // if (start_angle_ < 0) start_angle_ += 2 * M_PI;
        // if (end_angle_ >= 2 * M_PI) end_angle_ -= 2 * M_PI;
        
        // if((start_angle_ < end_angle_ && start_angle_ <= Robot_angle_ && Robot_angle_ <= end_angle_) || 
        //     (start_angle_ >= end_angle_ && (start_angle_ <= Robot_angle_ || 
        //     Robot_angle_ <= end_angle_)))
        if (abs(Robot_angle_) <= angle_area_ && Robot_distance_ <= radius_)
        {
            in_range.push_back(marker_ids_[i]);
        }
    } 
    ROS_INFO("observed landmarks num: %d", in_range.size());
}

//マーカー、パーティクル間誤差、尤度計算(距離、角度)
void PFVisualization::getLikelihood(size_t marker_id)
{
    const auto& marker = marker_positions_[marker_id];

    for (size_t j = 0; j < particle_positions_.size(); ++j)
    {
        const auto & particle = particle_positions_[j]; 

        dis_X_ = marker.position.x - particle.x;
        dis_Y_ = marker.position.y - particle.y;
        
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_);
        double particle_angle = particle.yaw - atan2(dis_Y_ , dis_X_);
        // if(particle_angle < 0){
        //     particle_angle += 2 * M_PI;
        // }
        
        double w_dis = 1/(sqrt(2 * M_PI * dis_var_))*exp(-((abs(Robot_distance_)-abs(particle_distance))*(abs(Robot_distance_)-abs(particle_distance)))/(2*dis_var_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Robot_angle_ - (- particle_angle - particle.yaw)) * ( Robot_angle_ - (- particle_angle - particle.yaw))) / (2 * ang_var_))+1e-100;
        
        if(Robot_angle_ * particle_angle > 0 && particle_angle > 1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Robot_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI))) * ( Robot_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI)))) / (2 * M_PI * ang_var_))+1e-100;
        }else if (Robot_angle_ * particle_angle > 0 && particle_angle < -1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Robot_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI))) * ( Robot_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI)))) / (2 * M_PI * ang_var_))+1e-100;
        }
        

        double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_dis_log + w_ang_log);
        
        Likelihood_[j]*=weight;

        //std::cout << "distance_weight" <<  w_dis << "angle_weight" << w_ang << "total_wight" << total_weight_ << :: std::endl;

        // std::cout << "Marker ID " << marker_ids_[i] << " and Particle " << j
        //           << " distance: (d: " << dis << ", angle: (a: " << ang << std::endl;
        // particle_weight << "distance_weight" << " " <<  w_dis_log << " " << "angle_weight" << " " << w_ang_log << " " << "wight" << " " << weight << " " << "total_wight" << total_weight_ << :: std::endl;

        // marker_particle_information << " " << "Marker ID " << " " << marker_ids_[marker_id] << " " << " and Particle " << " " << j
        //                             << " distance: (d: " << " " << particle_distance << " " << ", angle: (a: " << " " << particle_angle << std::endl;

        // for (const auto& info : Error_Information_) 
        // {
        // std::cout << "Angle: " << info.ang << ", Distance: " << info.dis << std::endl;
        // }
    }

    ROS_INFO_STREAM("Likelihood_ size: " << Likelihood_.size());
    ROS_INFO_STREAM("particle size: " <<  particle_positions_.size());
}   
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//重みの最適化過程、自己位置推定過程、リサンプリング過程--------------------------------------------------------------------------------------------------------------------------------------------
void PFVisualization::getEstimatedRobotPose()
{
    double Norm_total_weight = std::accumulate(Likelihood_.begin(), Likelihood_.end(), 0.0);
    double Particle_Est_RobotX = 0.0;
    double Particle_Est_RobotY = 0.0;
    double Particle_Est_RobotYaw = 0.0;

    ROS_INFO("Starting Localization_mode");
    
    for (size_t j = 0; j < particle_positions_.size(); ++j)
    {
        const auto & particle = particle_positions_[j];
        const auto & w = Likelihood_[j];
            
        // std::cout << " 正規化前尤度 " << w << std :: endl;


        Particle_Est_RobotX += particle.x * w;
        Particle_Est_RobotY += particle.y * w;
        Particle_Est_RobotYaw += particle.yaw * w;

        // std::cout << " 正規尤度 " << Norm_weight << std :: endl; 
        // particle_Norm_weight << "weight" << " " << w << " " << "total_weight" << " " << total_weight << " " << "Norm_weight" << " " << Norm_weight << " " << "Norm_total_weight" << " " << Norm_total_weight << std :: endl;

    }
    
    // std::cout << "=========" << std::endl;
    // std::cout << "Particle_Est_RobotX=" <<Particle_Est_RobotX<< std::endl;
    // std::cout << "Particle_Est_RobotY=" <<Particle_Est_RobotY<< std::endl;
    // std::cout << "Particle_Est_RobotTH=" <<Particle_Est_RobotYaw<< std::endl;
    ROS_INFO_STREAM("Norm total weight: " << Norm_total_weight <<
                    " EstX: " << Particle_Est_RobotX <<
                    " EstY: " << Particle_Est_RobotY <<
                    " EstTh: " << Particle_Est_RobotYaw);

    nav_msgs::Odometry est_msg;
    est_msg.header.stamp = ros::Time::now();
    est_msg.header.frame_id = odom_msg_.header.frame_id;
    est_msg.child_frame_id = odom_msg_.child_frame_id;
    est_msg.pose.pose = potbot_lib::utility::get_Pose(Particle_Est_RobotX,Particle_Est_RobotY,0,0,0,Particle_Est_RobotYaw);
    pub_estimated_robot_.publish(est_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    PFVisualization pfv;
    ros::spin();

    return 0;
}