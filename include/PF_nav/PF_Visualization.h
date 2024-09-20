#ifndef PF_VISUALIZATION_H_
#define PF_VISUALIZATION_H_

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

class PFVisualization
{
private:
    ros::Subscriber sub_marker_, sub_robot_pose_, sub_particle_pose_;
    ros::Publisher pub_estimated_robot_, pub_particles_, pub_particles_state_;

    nav_msgs::Odometry odom_msg_;
    std::vector<potbot_lib::DiffDriveAgent> particles_;

    //各記録用(ofstream定義)
    // std::ofstream record_end_angle_; //観測用カメラの測定最終角度
    // std::ofstream record_robot_pose_yaw; //ロボットの姿勢角度(観測では上記の2つの真ん中になっていれば良い)
    // std::ofstream record_angle; //ロボットとマーカーの角度(atan関数が機能しているかの確認)
    // std::ofstream particle_position; //各パーティクルの位置(動作モデル後)
    // std::ofstream marker_particle_information; //パーティクルとマーカー間の距離と角度(観測モデル使用情報)
    // std::ofstream particle_weight;//各パーティクルの尤度(距離、角度、尤度、合計)
    // std::ofstream particle_Norm_weight;//各パーティクルの正規化後の尤度(正規化前尤度、正規化前合計尤度、正規化前尤度、正規化前合計尤度(1になる))

    std::vector<geometry_msgs::Pose> marker_positions_; //マーカー位置定義
    std::vector<int> marker_ids_;; //マーカーid
    std::vector<double> Likelihood_; //各マーカの重み
    std::vector<double> Norm_Likelihood_;
    std::vector<double> step_sum_weight_;
    //std::vector<double> particle_position_x; //各パーティクルのx座標
    //std::vector<double> particle_position_y; //各パーティクルのx座標
    //std::vector<double> particle_position_yaw; //各パーティクルのx座標

    double robot_pose_x_ = 0.0, robot_pose_y_ = 0.0, robot_pose_z_ = 0.0, robot_pose_yaw_ = 0.0; //ロボット位置の真値(定義)
    double Robot_distance_ = 0.0, Robot_angle_ = 0.0; //ロボットとマーカーの距離と角度(角度に関してはロボット座標系における角度に変更する必要あるかも)
    double Scan_distance_ = 0.0, Scan_angle_ = 0.0; //ロボットから取得できるセンサ観測値
    double particle_position_x_ = 0.0, particle_position_y_ = 0.0, Particle_Est_RobotYaw_ = 0.0; //パーティクルの位置(使ってない)
    double dis_X_ = 0.0, dis_Y_ = 0.0; //各パーティクルの各マーカー間のX軸、Y軸誤差
    double radius_ = 3.0, start_angle_ = 0.0, end_angle_ = 0.0, angle_area_ = 85.2 * M_PI / 180; //ロボットの観測範囲の定義
    double dis_var_ = 0.2, ang_var_ = 0.2;
    //double dis_var_ = 0.000856, ang_var_ = 0.000464;
    double total_weight_ = 0.0;
    bool sebscribed_robot_pose_ = false;
    bool sebscribed_landmark_pose_ = false;
    bool sebscribed_particle_pose_ = false;
public:
    PFVisualization(/* args */);
    ~PFVisualization(){};

    void markerCallback(const visualization_msgs::MarkerArray& marker_array);
    void robotPoseCallback(const nav_msgs::Odometry& odom_pose);
    void updateParticles();

    void initLiklihood();
    void normLiklihood();
    void getObservedLandmark(std::vector<int>& in_range);
    void getLikelihood(size_t marker_id);
    void getEstimatedRobotPose();
    void localization();
     void getResamplingRobotPose0();
    void getResamplingRobotPose1(std::vector<double>& step_sum_weight_);
    void getResamplingRobotPose2(std::vector<double>& step_sum_weight_);
};

#endif