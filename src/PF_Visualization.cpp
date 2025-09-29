#include <PF_nav/PF_Visualization.h>

std::ofstream Likelihood_txt("/home/ros/catkin_ws/user/src/data/simulator/Likelihood.txt");
std::ofstream GA_Likelihood_txt("/home/ros/catkin_ws/user/src/data/simulator/GA_Likelihood.txt");
std::ofstream Ess_txt("/home/ros/catkin_ws/user/src/data/simulator/Ess.txt");
std::ofstream GA_Ess_txt("/home/ros/catkin_ws/user/src/data/simulator/GA_Ess.txt");
std::ofstream kakunin_Ess_sum_txt("/home/ros/catkin_ws/user/src/data/simulator/kakunin_Ess_sum.txt");
std::ofstream GA_kakunin_Ess_sum_txt("/home/ros/catkin_ws/user/src/data/simulator/GA_kakunin_Ess_sum.txt");
std::ofstream Estimate_position("/home/ros/catkin_ws/user/src/data/simulator/Estimate_position.csv");
std::ofstream Robot_command("/home/ros/catkin_ws/user/src/data/simulator/Robot_command.csv");
std::ofstream Pt_Position("/home/ros/catkin_ws/user/src/data/simulator/Pt_position.csv");
std::ofstream GA_Pt_Position("/home/ros/catkin_ws/user/src/data/simulator/GA_Pt_position.csv");
std::ofstream Pt_yaw("/home/ros/catkin_ws/user/src/data/simulator/Pt_yaw.txt");
std::ofstream Pt_yaw2("/home/ros/catkin_ws/user/src/data/simulator/Pt_yaw2.txt");
std::ofstream Robot_angle("/home/ros/catkin_ws/user/src/data/simulator/Robot_angle.csv");
std::ofstream Particle_angle("/home/ros/catkin_ws/user/src/data/simulator/Particle_angle.csv");
std::ofstream Particle_angle_2nd("/home/ros/catkin_ws/user/src/data/simulator/Particle_angle.csv");
std::ofstream CH_Particle("/home/ros/catkin_ws/user/src/data/simulator/CH_Particle.csv");
std::ofstream CL_Particle("/home/ros/catkin_ws/user/src/data/simulator/CL_Particle.csv");
std::ofstream CS_Particle("/home/ros/catkin_ws/user/src/data/simulator/CS_Particle.csv");
std::ofstream CM_Particle("/home/ros/catkin_ws/user/src/data/simulator/CM_Particle.csv");
std::ofstream CP_Particle("/home/ros/catkin_ws/user/src/data/simulator/CP_Particle.csv");
std::ofstream Marker_Scan("/home/ros/catkin_ws/user/src/data/simulator/Marker_Scan.txt");
std::ofstream Likelihood_Scan("/home/ros/catkin_ws/user/src/data/simulator/Likelihood_Scan.txt");
std::ofstream Element_yaw("/home/ros/catkin_ws/user/src/data/simulator/Element_yaw.txt");


int Pt_idx = 0;
int Robot_idx = 0;

PFVisualization::PFVisualization(/* args */)
{

    ros::NodeHandle n("~");

    int particle_num = 100;
	double norm_noise_mean_linear_velocity = 0;
	double norm_noise_variance_linear_velocity = 0.1;
	double norm_noise_mean_angular_velocity = 0;
	double norm_noise_variance_angular_velocity = 0.1;
	
	n.getParam("particle_num", particle_num);
	n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity);
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity);
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity);
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity);
    
    ros::NodeHandle nh;
	pub_particles_ = nh.advertise<geometry_msgs::PoseArray>("particles", 1);
    pub_estimated_robot_ = nh.advertise<nav_msgs::Odometry>("odom/estimated", 1);

    // サブスクライバの作成
    sub_marker_ = nh.subscribe("marker", 1000, &PFVisualization::markerCallback,this);
    sub_robot_pose_ = nh.subscribe("odom", 1000, &PFVisualization::robotPoseCallback,this);
    sub_robot_command_ = nh.subscribe("odom/truth", 1000, &PFVisualization::robotCommandCallback,this);
    sub_Dead_command_ = nh.subscribe("odom/dead_reckoning", 1000, &PFVisualization::robotDeadCallback,this);


	std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution_linear_velocity(norm_noise_mean_linear_velocity, sqrt(norm_noise_variance_linear_velocity));
	std::normal_distribution<double> distribution_angular_velocity(norm_noise_mean_angular_velocity, sqrt(norm_noise_variance_angular_velocity));

    particles_.resize(particle_num);
    for (auto& p : particles_)
	{
		// robo.deltatime = 1.0/control_frequency;
        nav_msgs::Odometry p_msg;
        p.x = distribution_linear_velocity(generator);
        p.y = distribution_linear_velocity(generator);
        p.yaw = distribution_angular_velocity(generator);
	}
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
        
    }
}


void PFVisualization::robotCommandCallback(const nav_msgs::Odometry& odom_true_pose)
{
    robot_velocity_ =  odom_true_pose.twist.twist.linear.x;
    robot_angular_velocity_ = odom_true_pose.twist.twist.angular.z;
}
//ロボットのコールバック関数
void PFVisualization::robotPoseCallback(const nav_msgs::Odometry& odom_pose)
{
    sebscribed_robot_pose_ = true;
    odom_msg_ = odom_pose;
    robot_pose_x_ = odom_pose.pose.pose.position.x;
    robot_pose_y_ = odom_pose.pose.pose.position.y;
    robot_pose_z_ = odom_pose.pose.pose.position.z;

    robot_pose_yaw_ = tf2::getYaw(odom_pose.pose.pose.orientation);
    
    if (sebscribed_landmark_pose_ && sebscribed_robot_pose_)
    {
        localization();
    }    
    
}

void PFVisualization::robotDeadCallback(const nav_msgs::Odometry& odom_dead_pose)
{
    robot_dead_velocity_ = odom_dead_pose.twist.twist.linear.x;
    robot_dead_angular_velocity_ = odom_dead_pose.twist.twist.angular.z;
}

void PFVisualization::localization()
{
    static ros::Time last_time = ros::Time::now();  // 前ループの時刻を保持
    ros::Time now = ros::Time::now();    
    static int Loop_count = 0;  // ループ回数カウンタ
    const int PF_Localization_Interval = 5; // 5回に1回PF更新

    double dt = (now - last_time).toSec();         // 経過時間を秒で計算
    last_time = now;

    if (initial_Time) {
        initialparticlepose();
    }

    filteringdecision();
    
    if(subscribed_robot_command){
        
        updateParticles();

        Loop_count++;

        std::vector<int> in_range_ids;

        bool Localization_PF = false;
        if(Loop_count >= PF_Localization_Interval)
        {
            Count_step_ = Count_step_ + 1;
            Likelihood_txt <<  " 実行回数 "  << Count_step_ << std :: endl;
            GA_Likelihood_txt <<  " 実行回数 "  << Count_step_ << std :: endl;
            Pt_Position <<  " 実行回数 "  << Count_step_ <<  " 実行回数 "  << Count_step_ <<  " 実行回数 "  << Count_step_ << std :: endl;
            GA_Pt_Position <<  " 実行回数 "  << Count_step_ <<  " 実行回数 "  << Count_step_ <<  " 実行回数 "  << Count_step_ << std :: endl;

            Loop_count = 0;
            Localization_PF = true;
            
            Scan_Count_ += 1;
            getObservedLandmark(observed_markers);
            initLiklihood();
            
            Likelihood_Scan << "---Likelihood Scan 開始---" << "実行回数:" << Scan_Count_ << std::endl;
            for (const auto& in_range_Marker:observed_markers)
            {
                getLikelihood_main(in_range_Marker);
            }
            Likelihood_Scan << "---Likelihood Scan 終了---" << std::endl;

            normLiklihood();

            getEstimatedRobotPose2(Localization_PF,dt);

            // AdaptiveGeneticAlgorithm();

            // for (const auto& in_range_Marker:observed_markers)
            // {
            //     getLikelihood_2nd(in_range_Marker);
            // }

            // normLiklihood();

            getResamplingRobotPose1(step_sum_weight_);
            
        }else{
            getEstimatedRobotPose2(Localization_PF,dt);
        }
    }else{
        // std::cout << "not robot command" << std::endl;
    }

}

void PFVisualization::filteringdecision()
{   
    Robot_command <<  " 速度 "  <<  robot_velocity_ << " 角速度 " << robot_angular_velocity_ << std::endl;

    if ( robot_velocity_ == 0 && robot_angular_velocity_ == 0 )
    {
        subscribed_robot_command = false;
    } else {
        subscribed_robot_command = true;
    }
}

void PFVisualization::initialparticlepose()
{
    ros::NodeHandle n("~");

    double initial_particle_Position_mean_noise = 0;
    double initial_particle_Position_variance_noise = 0.1;
    double initial_particle_yaw_mean_noise = 0;
    double initial_particle_yaw_variance_noise = 0.1;

    n.getParam("initial_particle_Position_mean_noise", initial_particle_Position_mean_noise);
    n.getParam("initial_particle_Position_variance_noise", initial_particle_Position_variance_noise);
    n.getParam("initial_particle_yaw_mean_noise", initial_particle_yaw_mean_noise);
    n.getParam("initial_particle_yaw_variance_noise", initial_particle_yaw_variance_noise);

    std::random_device  initial_rd;
    std::default_random_engine generator(initial_rd());
    std::normal_distribution<double> initial_particle_Position_noise(initial_particle_Position_mean_noise, sqrt(initial_particle_Position_variance_noise));
	std::normal_distribution<double> initial_particle_yaw_noise(initial_particle_yaw_mean_noise, sqrt(initial_particle_yaw_variance_noise));

    for (auto& p : particles_)
	{
        p.x = p.x + initial_particle_Position_noise(generator);
        p.y = p.y + initial_particle_Position_noise(generator);
        p.yaw =  p.yaw  + initial_particle_yaw_noise(generator);
        // std::cout << p.x << p.y << p.yaw << std::endl;
	}
    
    // std::cout << "パーティクル拡散" << std::endl;
    initial_Time = false;    
}

void PFVisualization::updateParticles()
{
    size_t yaw_out_of_range_count = 0;  // 呼び出しごとにリセット
    size_t yaw_out_of_range_count2 = 0;  // 呼び出しごとにリセット
    Assessment_count_ += 1;

    double sum_theta = 0.0;
    double sum_theta2 = 0.0;
    geometry_msgs::PoseArray particles_msg;
    Pt_yaw << " ---yaw角査定 " << Assessment_count_ << " 回目---" << std::endl;
    Pt_yaw2 << " ---yaw角査定 " << Assessment_count_ << " 回目---" << std::endl;


    for (auto& p : particles_)
    {
        double v = odom_msg_.twist.twist.linear.x;
        double omega = odom_msg_.twist.twist.angular.z;
        p.v = v;
        p.omega = omega;
        p.deltatime = 1.0 / 50.0;
        p.update();

        double theta = p.yaw;
        sum_theta += theta;

        // 範囲チェックのみ
        if (!(theta >= -M_PI && theta < M_PI))
        {
            yaw_out_of_range_count++;
            Pt_yaw << " [警告] yaw角が範囲外: " << theta << std::endl;
            // 正規化処理 [-π, π) に収める
            while (theta >= M_PI) theta -= 2.0 * M_PI;
            while (theta < -M_PI) theta += 2.0 * M_PI;

            p.yaw = theta;  // 正規化後の値を代入
        }

        double theta2 = p.yaw;

        if (!(theta2 >= -M_PI && theta2 < M_PI))
        {
            yaw_out_of_range_count2++;
            Pt_yaw2 << " [警告] yaw角が範囲外: " << theta2 << std::endl;
        }
        
        sum_theta2 += theta2;
        
        nav_msgs::Odometry p_msg;
        potbot_lib::utility::to_msg(p, p_msg);
        particles_msg.poses.push_back(p_msg.pose.pose);
    }

    particles_msg.header.frame_id = "map";
    particles_msg.header.stamp = ros::Time::now();
    pub_particles_.publish(particles_msg);

    avg_theta = sum_theta / particles_.size();
    avg_theta2 = sum_theta2 / particles_.size();
    Pt_yaw << " ---yaw角査定終了--- "
           << " パーティクル平均角度 " << avg_theta
           << " (範囲外カウント: " << yaw_out_of_range_count << ")\n";

    Pt_yaw2 << " ---yaw角査定終了--- "
           << " パーティクル平均角度 " << avg_theta2
           << " (範囲外カウント: " << yaw_out_of_range_count2 << ")\n";
}

void PFVisualization::initLiklihood()
{
    Likelihood_.clear();
    Likelihood_.resize(particles_.size());
    std::fill(Likelihood_.begin(), Likelihood_.end(), 1.0 / particles_.size());
}

void PFVisualization::normLiklihood()
{
    double sum = std::accumulate(Likelihood_.begin(), Likelihood_.end(), 0.0);
    for(auto& val:Likelihood_) val/=sum;
}

double PFVisualization::wrapAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

//範囲内マーカー観測プロセス
void PFVisualization::getObservedLandmark(std::vector<ObservedMarker>& observed_markers)
{
    ros::NodeHandle n("~");

    double norm_noise_mean_Scan_distance = 0;
    double norm_noise_variance_Scan_distance = 0.1;
    double norm_noise_mean_Scan_angle = 0;
    double norm_noise_variance_Scan_angle = 0.1;
    double norm_noise_mean_Scan_Long_distance = 0;
    double norm_noise_variance_Scan_Long_distance = 0;

    n.getParam("norm_noise_mean_Scan_distance", norm_noise_mean_Scan_distance);
    n.getParam("norm_noise_variance_Scan_distance", norm_noise_variance_Scan_distance);
    n.getParam("norm_noise_mean_Scan_angle", norm_noise_mean_Scan_angle);
    n.getParam("norm_noise_variance_Scan_angle", norm_noise_variance_Scan_angle);
    n.getParam("norm_noise_mean_Scan_Long_distance", norm_noise_mean_Scan_Long_distance);
    n.getParam("norm_noise_variance_Scan_Long_distance", norm_noise_variance_Scan_Long_distance);
     
    std::random_device rd2;
    std::default_random_engine generator(rd2());
    std::normal_distribution<double> distribution_Scan_distance(norm_noise_mean_Scan_distance, sqrt(norm_noise_variance_Scan_distance));
    std::normal_distribution<double> distribution_Scan_angle(norm_noise_mean_Scan_angle, sqrt(norm_noise_variance_Scan_angle));
    std::normal_distribution<double> distribution_Scan_Long_distance(norm_noise_mean_Scan_Long_distance, sqrt(norm_noise_variance_Scan_Long_distance));

    robot_distances_.clear();
    robot_scan_distances_.clear();
    robot_angles_.clear();
    robot_scan_angles_.clear();
    observed_markers.clear();

    double Robot_atan = 0.0;

    Element_yaw << "---角度確認 開始---" << "実行回数" << Scan_Count_ << std::endl;
    Element_yaw << " パーティクル平均角度: " << avg_theta 
                << " ロボット角度: " << robot_pose_yaw_ << std::endl;
    Element_yaw << "---角度確認 終了---" << std::endl;

    Marker_Scan << "---Marker Scan 開始---" << "実行回数" << Scan_Count_ << std::endl;

    
    auto inFOV = [&](double marker_angle, double robot_yaw, double fov_rad) {
        double start_angle = wrapAngle(robot_yaw - fov_rad / 2.0);
        double end_angle   = wrapAngle(robot_yaw + fov_rad / 2.0);
        marker_angle = wrapAngle(marker_angle);

        if (start_angle <= end_angle) {
            return (marker_angle >= start_angle && marker_angle <= end_angle);
        } else {
            // 2πを跨ぐ場合
            return (marker_angle >= start_angle || marker_angle <= end_angle);
        }
    };

    // ---- マーカごとの処理 ----
    for(size_t i = 0; i < marker_positions_.size(); ++i)
    {
        const auto& marker_pose = marker_positions_[i];
        double dx = marker_pose.position.x - robot_pose_x_;
        double dy = marker_pose.position.y - robot_pose_y_;

        Robot_distance_ = std::sqrt(dx * dx + dy * dy);
        robot_distances_.push_back(Robot_distance_);

        // 距離に応じてノイズを付与
        if(Robot_distance_ <= 5.0){
            Scan_distance_ = Robot_distance_ + distribution_Scan_distance(generator);
        } else if(Robot_distance_ > 5.0 &&  Robot_distance_ <= 8.0){
            Scan_distance_ = Robot_distance_ + distribution_Scan_Long_distance(generator);
        } else {
            Scan_distance_ = 0;
        }
        robot_scan_distances_.push_back(Scan_distance_);

        if(Robot_distance_ > radius_){
            continue;
        }
        
        // マーカ方向（世界座標系 → ロボット基準角度）
        Robot_atan   = std::atan2(dy, dx);
        Robot_angle_ = wrapAngle(Robot_atan - robot_pose_yaw_);

        Robot_idx += 1;
        Robot_angle << " マーカ番号 "  << marker_ids_[i] 
                    << " atan角度 "    << Robot_atan 
                    << " ロボット姿勢 " << robot_pose_yaw_ 
                    << " 計算後の尤度角度 " << Robot_angle_ 
                    << " 処理番号 " << Robot_idx << std::endl;
        
        robot_angles_.push_back(Robot_angle_);

        // 角度ノイズを加える
        Scan_angle_ = Robot_angle_ + distribution_Scan_angle(generator);
        robot_scan_angles_.push_back(Scan_angle_);

        // ---- 視野判定 ----
        if (inFOV(Robot_atan, robot_pose_yaw_, angle_area_) && Robot_distance_ <= radius_) {
            Marker_Scan << " マーカID: " << marker_ids_[i] 
                        << " スキャン距離: " << Scan_distance_ 
                        << " スキャン角度: " << Scan_angle_ << std::endl;
            ObservedMarker m;
            m.id = marker_ids_[i];
            m.distance = Scan_distance_;
            m.angle = Scan_angle_;
            observed_markers.push_back(m);
        }
    } 

    Marker_Scan << "---Marker Scan 終了---" << std::endl;
}

//マーカー、パーティクル間誤差、従来法による尤度計算(距離、角度)
void PFVisualization::getLikelihood(size_t marker_id)
{
    const auto& marker = marker_positions_[marker_id];
    double Scan_distance_ = robot_scan_distances_[marker_id];
    double Scan_angle_ = robot_scan_angles_[marker_id];
    double Pt_atan = 0.0;
    Pt_idx += 1;

    for (size_t j = 0; j < particles_.size(); ++j)
    {
        const auto & particle = particles_[j]; 

        dis_X_ = marker.position.x - particle.x; //マーカとパーティクルのx座標距離(予測値の距離：推定値)
        dis_Y_ = marker.position.y - particle.y; //マーカとパーティクルのy座標距離(予測値の距離：推定値)
        
        Pt_atan = atan2(dis_Y_ , dis_X_);

        double Local_dis_ = 0.0;
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_); //マーカとパーティクルの直線距離
        double particle_angle = atan2(dis_Y_ , dis_X_) - particle.yaw ; //マーカとパーティクルの角度

        
        Particle_angle <<  " マーカ番号 "  << marker_ids_[marker_id] <<  " atan角度 "  << Pt_atan << " パーティクルID " << j <<  " パーティクル姿勢 "  << particle.yaw << " 計算後の尤度角度 "  << particle_angle << " 処理番号 " << Pt_idx << std :: endl;

        Local_dis_ = dis_var_ * dis_X_ * dis_X_;  //尤度関数分散値の変更式(実機の方に実装されている分散はこっち)

        // std::cout << "スキャン距離" << Scan_distance_ << "パーティクル距離" << particle_distance << std::endl;

        double w_dis = 1/(sqrt(2 * M_PI * Local_dis_))*exp(-((abs(Scan_distance_)-abs(particle_distance))*(abs(Scan_distance_)-abs(particle_distance)))/(2*Local_dis_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ -  particle_angle ) * ( Scan_angle_ - particle_angle )) / (2 * ang_var_))+1e-100;

        if (Scan_distance_ == 0)
        {
            w_dis = 1;
        }
        
        if (Scan_angle_ == 0)
        {
            w_ang = 1;
        }

        if (abs(abs(Scan_distance_)-abs(particle_distance))>1.3)
        {
            w_dis=1;
        }
        
        double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_dis_log + w_ang_log);

        Likelihood_[j]*=weight;
    }
}   

//マーカー、パーティクル間誤差、提案法による尤度計算(距離、角度)
void PFVisualization::getLikelihood_main(const ObservedMarker& in_range_Marker)
{
    const auto& marker = marker_positions_[in_range_Marker.id];
    double Scan_distance_ = in_range_Marker.distance;
    double Scan_angle_ = in_range_Marker.angle;
    double Pt_atan = 0.0;
    Pt_idx += 1;

    Likelihood_Scan << " マーカID: " << marker_ids_[in_range_Marker.id] << " スキャン距離: " << Scan_distance_ << " スキャン角度: " << Scan_angle_ << std::endl;

    for (size_t j = 0; j < particles_.size(); ++j)
    {
        const auto & particle = particles_[j]; 

        dis_X_ = marker.position.x - particle.x; //マーカとパーティクルのx座標距離(予測値の距離：推定値)
        dis_Y_ = marker.position.y - particle.y; //マーカとパーティクルのy座標距離(予測値の距離：推定値)

        Pt_atan = atan2(dis_Y_ , dis_X_);
        
        double Local_dis_ = 0.0;
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_);
        double particle_angle_atan = atan2(dis_Y_, dis_X_) - particle.yaw;
        double particle_angle = wrapAngle(Scan_angle_ - particle_angle_atan);
        
        Particle_angle <<  " マーカ番号 "  << marker_ids_[in_range_Marker.id] <<  " atan角度 "  << Pt_atan << " パーティクルID " << j <<  " パーティクル姿勢 "  << particle.yaw << " 計算後の尤度角度 "  << particle_angle << " 処理番号 " << Pt_idx << std :: endl;

        Local_dis_ = dis_var_ * dis_X_ * dis_X_;  //尤度関数分散値の変更式(実機の方に実装されている分散はこっち)
        
        //変曲点に着目した距離分散変動
        observe_scan_distance_error_ = abs(abs(Scan_distance_)-abs(particle_distance));

        if (abs(abs(Scan_distance_)-abs(particle_distance))>0.8&&abs(abs(Scan_distance_)-abs(particle_distance))<1.3)
        {
            Local_dis_=abs(abs(Scan_distance_)-abs(particle_distance))*abs(abs(Scan_distance_)-abs(particle_distance));
            // Scan_distance_ = Scan_distance_ - 0.80; (2025-04-30なんでこの工程を入れたので残しておきます)
        }

        // std::cout << "スキャン距離" << Scan_distance_ << "パーティクル距離" << particle_distance << std::endl;

        double w_dis = 1/(sqrt(2 * M_PI * Local_dis_))*exp(-((abs(Scan_distance_)-abs(particle_distance))*(abs(Scan_distance_)-abs(particle_distance)))/(2*Local_dis_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(particle_angle * particle_angle) / (2 * ang_var_))+1e-100;

        if (Scan_distance_ == 0)
        {
            w_dis = 1;
        }
        
        if (Scan_angle_ == 0)
        {
            w_ang = 1;
        }

        if (abs(abs(Scan_distance_)-abs(particle_distance))>1.3)
        {
            w_dis=1;
        }
        
        double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_dis_log + w_ang_log);

        Likelihood_[j]*=weight;
        Likelihood_txt << "尤度" << " " << Likelihood_[j] << std::endl;        
    }
}

void PFVisualization::getLikelihood_2nd(const ObservedMarker& in_range_Marker)
{
    const auto& marker = marker_positions_[in_range_Marker.id];
    double Scan_distance_ = in_range_Marker.distance;
    double Scan_angle_ = in_range_Marker.angle;
    double Pt_atan = 0.0;
    Pt_idx += 1;

    for (size_t j = 0; j < particles_.size(); ++j)
    {
        const auto & particle = particles_[j]; 

        dis_X_ = marker.position.x - particle.x; //マーカとパーティクルのx座標距離(予測値の距離：推定値)
        dis_Y_ = marker.position.y - particle.y; //マーカとパーティクルのy座標距離(予測値の距離：推定値)

        Pt_atan = atan2(dis_Y_ , dis_X_);
        
        double Local_dis_ = 0.0;
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_);
        double particle_angle_atan = atan2(dis_Y_, dis_X_) - particle.yaw;
        double particle_angle = wrapAngle(Scan_angle_ - particle_angle_atan);
        
        Particle_angle_2nd <<  " マーカ番号 "  << marker_ids_[in_range_Marker.id] <<  " atan角度 "  << Pt_atan << " パーティクルID " << j <<  " パーティクル姿勢 "  << particle.yaw << " 計算後の尤度角度 "  << particle_angle << " 処理番号 " << Pt_idx << std :: endl;

        Local_dis_ = dis_var_ * dis_X_ * dis_X_;  //尤度関数分散値の変更式(実機の方に実装されている分散はこっち)
        
        //変曲点に着目した距離分散変動
        observe_scan_distance_error_ = abs(abs(Scan_distance_)-abs(particle_distance));

        if (abs(abs(Scan_distance_)-abs(particle_distance))>0.8&&abs(abs(Scan_distance_)-abs(particle_distance))<1.3)
        {
            Local_dis_=abs(abs(Scan_distance_)-abs(particle_distance))*abs(abs(Scan_distance_)-abs(particle_distance));
            // Scan_distance_ = Scan_distance_ - 0.80; (2025-04-30なんでこの工程を入れたので残しておきます)
        }

        // std::cout << "スキャン距離" << Scan_distance_ << "パーティクル距離" << particle_distance << std::endl;

        double w_dis = 1/(sqrt(2 * M_PI * Local_dis_))*exp(-((abs(Scan_distance_)-abs(particle_distance))*(abs(Scan_distance_)-abs(particle_distance)))/(2*Local_dis_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(particle_angle * particle_angle) / (2 * ang_var_))+1e-100;

        if (Scan_distance_ == 0)
        {
            w_dis = 1;
        }
        
        if (Scan_angle_ == 0)
        {
            w_ang = 1;
        }

        if (abs(abs(Scan_distance_)-abs(particle_distance))>1.3)
        {
            w_dis=1;
        }
        
        double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_dis_log + w_ang_log);

        Likelihood_[j]*=weight;
        GA_Likelihood_txt << "尤度" << " " << Likelihood_[j] << std::endl;
    }
}

void PFVisualization::getEstimatedRobotPose2(bool Localization_PF, double dt)
{
    nav_msgs::Odometry est_msg;
    est_msg.header.stamp = ros::Time::now();
    est_msg.header.frame_id = odom_msg_.header.frame_id;
    est_msg.child_frame_id = odom_msg_.child_frame_id;

    double PF_Estimate_position_x_ = 0.0;
    double PF_Estimate_position_y_ = 0.0;

    if (Localization_PF) {
        // === PFで推定 ===
        double sin_sum = 0.0;
        double cos_sum = 0.0;

        for (size_t j = 0; j < particles_.size(); ++j) { 
            Pt_Position << "Pt_X" << " " << particles_[j].x << " " << "Pt_Y" << " " << particles_[j].y << " " << "Pt_Yaw" << " "  << particles_[j].yaw << " " << std::endl;
            PF_Estimate_position_x_ += particles_[j].x * Likelihood_[j];
            PF_Estimate_position_y_ += particles_[j].y * Likelihood_[j];
            sin_sum += std::sin(particles_[j].yaw) * Likelihood_[j];
            cos_sum += std::cos(particles_[j].yaw) * Likelihood_[j];
        }

        est_robot_pose_x_ = PF_Estimate_position_x_;
        est_robot_pose_y_ = PF_Estimate_position_y_;
        est_robot_pose_yaw_ = std::atan2(sin_sum, cos_sum);  // ベクトル平均で安定化

        est_msg.pose.pose = potbot_lib::utility::get_pose(
            est_robot_pose_x_, est_robot_pose_y_, 0, 0, 0, est_robot_pose_yaw_);

        Estmate_Count_ += 1;
        PF_Estimate_Count_ += 1;
        ROS_INFO_STREAM("--- Localization by Particle Filter --- " << Estmate_Count_ << "--- PF_Estimate_count --- " <<  PF_Estimate_Count_);

    } else {
        // === デッドレコニング ===
        est_robot_pose_x_   += robot_dead_velocity_ * std::cos(est_robot_pose_yaw_) * dt;
        est_robot_pose_y_   += robot_dead_velocity_ * std::sin(est_robot_pose_yaw_) * dt;
        est_robot_pose_yaw_ += robot_dead_angular_velocity_ * dt;

        est_msg.pose.pose = potbot_lib::utility::get_pose(
            est_robot_pose_x_, est_robot_pose_y_, 0, 0, 0, est_robot_pose_yaw_);

        Estmate_Count_ += 1;
        ROS_INFO_STREAM("--- Localization by Dead Reckoning --- " << Estmate_Count_);
    }

    pub_estimated_robot_.publish(est_msg);
}

//重みの最適化過程、自己位置推定過程、リサンプリング過程--------------------------------------------------------------------------------------------------------------------------------------------
void PFVisualization::getEstimatedRobotPose()
{
    double Norm_total_weight = std::accumulate(Likelihood_.begin(), Likelihood_.end(), 0.0);
    double Particle_Est_RobotX = 0.0;
    double Particle_Est_RobotY = 0.0;
    double Particle_Est_RobotYaw = 0.0;

    for (size_t j = 0; j < particles_.size(); ++j)
    {
        const auto & particle = particles_[j];
        const auto & w = Likelihood_[j];

        Particle_Est_RobotX += particle.x * w;
        Particle_Est_RobotY += particle.y * w;
        Particle_Est_RobotYaw += particle.yaw * w;
    }

    // --- 平滑化 ---
    est_x_history.push_back(Particle_Est_RobotX);
    est_y_history.push_back(Particle_Est_RobotY);
    est_yaw_history.push_back(Particle_Est_RobotYaw);

    if (est_x_history.size() > smoothing_window_size)
    {
        est_x_history.pop_front();
        est_y_history.pop_front();
        est_yaw_history.pop_front();
    }

    double smoothed_x = std::accumulate(est_x_history.begin(), est_x_history.end(), 0.0) / est_x_history.size();
    double smoothed_y = std::accumulate(est_y_history.begin(), est_y_history.end(), 0.0) / est_y_history.size();
    double smoothed_yaw = std::accumulate(est_yaw_history.begin(), est_yaw_history.end(), 0.0) / est_yaw_history.size();

    // 推定位置の出力
    nav_msgs::Odometry est_msg;
    est_msg.header.stamp = ros::Time::now();
    est_msg.header.frame_id = odom_msg_.header.frame_id;
    est_msg.child_frame_id = odom_msg_.child_frame_id;
    est_msg.pose.pose = potbot_lib::utility::get_pose(smoothed_x, smoothed_y, 0, 0, 0, smoothed_yaw);
    pub_estimated_robot_.publish(est_msg);
}

void PFVisualization::AdaptiveGeneticAlgorithm()
{
    ros::NodeHandle n("~");

    struct ParticleWithLikelihood {
    potbot_lib::DiffDriveAgent particle;  // 元のパーティクル
    double likelihood;                     // パーティクルの尤度
    int index;                             // 元のインデックス
    int label = 0;
    };

    std::vector<ParticleWithLikelihood> particles_tmp;  //一時的なパーティクル情報の配列(定義)
    std::vector<ParticleWithLikelihood> CL;  //重みの小さなパーティクル群
    std::vector<ParticleWithLikelihood> CH;  //重みの大きなパーティクル群
    std::vector<ParticleWithLikelihood> CS;  //補正したパーティクル群
    std::vector<ParticleWithLikelihood> CM;  //補正したパーティクル群
    std::vector<ParticleWithLikelihood> CP;
    double Am = 0.1;  //交叉パラメータ(論文では0.1)
    double RL = 0.0;  //突然変異確率の閾値
    double Pm = 0.5;  //突然変異確率(論文では0.5)
    double Tw = 0.001;  //有効サンプル数より決定される閾値
    double ESS_sum = 0.0;  //重みの2乗和
    double GA_kakunin_Ess_sum = 0.0;
    double Effective_Sample_Size = 0.0;  //有効サンプル数
    int Count_CH = 0;
    int Count_CL = 0;
    int Count_CS = 0;
    int Count_CM = 0;
    int Count_CP = 0;


    for(size_t i = 0; i < particles_.size(); i++)
    {
        particles_tmp.push_back({particles_[i],Likelihood_[i],static_cast<int>(i)});
    }
  
    std::sort(particles_tmp.begin(),particles_tmp.end(),[](const ParticleWithLikelihood &a, const ParticleWithLikelihood &b){
        return a.likelihood < b.likelihood;
        }
    );
    
    
    for (const auto &p : particles_tmp) 
    {
        GA_kakunin_Ess_sum += p.likelihood;
        ESS_sum += p.likelihood * p.likelihood;
    }
    
    GA_kakunin_Ess_sum_txt << "合計" << GA_kakunin_Ess_sum << std::endl;
    Effective_Sample_Size = 1 / ESS_sum;
    GA_Ess_txt << "Effective_Sample_Size_  " << Effective_Sample_Size << std::endl;

    size_t ess_index = static_cast<size_t>(Effective_Sample_Size);
    if (ess_index >= particles_tmp.size()) ess_index = particles_tmp.size() - 1;
    Tw = particles_tmp[ess_index].likelihood;

    for (auto &p : particles_tmp) {
        if (p.likelihood <= Tw) {
            p.label = -1;
            CL.push_back(p);
        } else {
           p.label = 1;
           CH.push_back(p);
        }
    } 
     
    CH_Particle << "=== CHgroup (label=1) ===  " <<  "Tw = " << Tw << std :: endl;
    for (const auto &p : CH) 
    {
        CH_Particle << "idx=" << p.index << " likelihood=" << p.likelihood << " label=" << p.label << std :: endl;
        Count_CH += 1;
    }
    CH_Particle << "=== Count_CH ===" << Count_CH << std :: endl;
    

    CL_Particle << "=== CLgroup (label=0) ===  " <<  "Tw = " << Tw << std :: endl;
    for (const auto &p : CL) 
    {
        CL_Particle << "idx=" << p.index << " likelihood=" << p.likelihood << " label=" << p.label << std :: endl;
        Count_CL += 1;
    }
    CL_Particle << "=== Count_CL ===" << Count_CL << std :: endl;

    for (auto &p : CL){

        static std::mt19937 rng(
        static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count())
        );

        if(!CH.empty()){
            std::uniform_int_distribution<std::size_t> dist(0, CH.size() - 1);
            std::size_t idx = dist(rng);

             // CH からランダムに 1 つ抽出（削除はしない）
            p.particle.x = Am * p.particle.x + (1 - Am) * CH[idx].particle.x;
            p.particle.y = Am * p.particle.y + (1 - Am) * CH[idx].particle.y;
            p.particle.yaw = wrapAngle(Am * p.particle.yaw + (1 - Am) * CH[idx].particle.yaw);
        }

        CS.push_back(p);
    }

    CS_Particle << "=== CSgroup ===  " << std :: endl;
    for (const auto &p : CS) 
    {
        CS_Particle << "PositionX" << p.particle.x << "PositionY" << p.particle.y << "PositionTH" << p.particle.yaw <<  "idx" << p.index <<  std :: endl;
        Count_CS += 1;
    }
    CS_Particle << "=== Count_CS ===" << Count_CS << std :: endl;
    
    for (auto &p : CS){

        std::mt19937 rng(std::random_device{}());

        std::uniform_real_distribution<double> uni01(0.0, 1.0);
        RL = uni01(rng);
        
        if(!CH.empty()){
            std::uniform_int_distribution<size_t> dist(0, CH.size()-1);
            size_t idx = dist(rng);
            if (RL <= Pm){
                p.particle.x = 2 * CH[idx].particle.x - p.particle.x;
                p.particle.y = 2 * CH[idx].particle.y - p.particle.y;
                p.particle.yaw = wrapAngle(2 * CH[idx].particle.yaw - p.particle.yaw);
            }else{
                p.particle.x =  p.particle.x;
                p.particle.y =  p.particle.y;
                p.particle.yaw =  p.particle.yaw;
            }
        }
        CM.push_back(p);
    }

    CM_Particle << "=== CMgroup ===  " << std :: endl;
    for (const auto &p : CM) 
    {
        CM_Particle << "PositionX" << p.particle.x << "PositionY" << p.particle.y << "PositionTH" << p.particle.yaw <<  "idx" << p.index <<  std :: endl;
        Count_CM += 1;
    }
    CM_Particle << "=== Count_CM ===" << Count_CM << std :: endl;

    CP.reserve(CH.size() + CM.size());

    // CH と CS を CP にコピー
    CP.insert(CP.end(), CH.begin(), CH.end());
    CP.insert(CP.end(), CM.begin(), CM.end());
    
    std::sort(CP.begin(), CP.end(), [](const ParticleWithLikelihood &a, const ParticleWithLikelihood &b) {
    return a.index < b.index;  // idx が小さい順
    });

    CP_Particle << "=== CPgroup ===  " << std :: endl;
    for (const auto &p : CP) 
    {
        CP_Particle << "idx=" << p.index <<  " label="  << p.label <<  std :: endl;
        Count_CP += 1;
    }
    CP_Particle << "=== Count_CP ===" << Count_CP << std :: endl;

    for (size_t i = 0; i < CP.size() && i < particles_.size(); i++) 
    {
        particles_[i].x   = CP[i].particle.x;
        particles_[i].y   = CP[i].particle.y;
        particles_[i].yaw = wrapAngle(CP[i].particle.yaw);
        GA_Pt_Position << "Pt_X" << " " << particles_[i].x << " " << "Pt_Y" << " " << particles_[i].y << " " << "Pt_Yaw" << " "  << particles_[i].yaw << " " << std::endl;
    }
}

//最大尤度を用いたリサンプリング方式
void PFVisualization::getResamplingRobotPose0()
{
    ros::NodeHandle n("~");
    
    double norm_noise_mean_linear_velocity = 0;
	double norm_noise_variance_linear_velocity = 0.1;
	double norm_noise_mean_angular_velocity = 0;
	double norm_noise_variance_angular_velocity = 0.1;

    n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity);
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity);
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity);
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity);   

    std::vector<potbot_lib::DiffDriveAgent> particles_tmp = particles_;
    int Max_Likelihood_idx = 0;

    double step_weight = Likelihood_[0];

    particles_[0].x = particles_tmp[0].x;
    particles_[0].y = particles_tmp[0].y;
    particles_[0].yaw = particles_tmp[0].yaw;
   
    for (size_t i = 1; i < particles_.size(); ++i)
	{
		if(step_weight < Likelihood_[i])
        {
            Max_Likelihood_idx = i;
        
            particles_[0].x = particles_tmp[i].x;
            particles_[0].y = particles_tmp[i].y;
            particles_[0].yaw = particles_tmp[i].yaw;
        }
	}
    
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution_linear_velocity(norm_noise_mean_linear_velocity, sqrt(norm_noise_variance_linear_velocity));
	std::normal_distribution<double> distribution_angular_velocity(norm_noise_mean_angular_velocity, sqrt(norm_noise_variance_angular_velocity));


    for (size_t j = 1; j < particles_.size(); ++j)
	{
        
        particles_[j].x = particles_[0].x + distribution_linear_velocity(generator);
        particles_[j].y = particles_[0].y + distribution_linear_velocity(generator);
        particles_[j].yaw = particles_[0].yaw + distribution_angular_velocity(generator);
	}

}

//リサンプリング(鈴木ver)
void PFVisualization::getResamplingRobotPose1(std::vector<double>& step_sum_weight_)
{
    // --- 1. 累積和とESS計算 ---
    step_sum_weight_.clear();
    double step_weight = 0.0;
    double ESS_sum = 0.0;
    double kakunin_Ess_sum = 0.0;
    
    for (size_t i = 0; i < particles_.size(); ++i)
    {
        kakunin_Ess_sum += Likelihood_[i];
        step_weight += Likelihood_[i];
        ESS_sum += Likelihood_[i] * Likelihood_[i];
        step_sum_weight_.push_back(step_weight);
    }
    
    kakunin_Ess_sum_txt << "合計" << kakunin_Ess_sum << std::endl;
    double Effective_Sample_Size = 1.0 / ESS_sum;
    Ess_txt << "Effective_Sample_Size_  " << Effective_Sample_Size << std::endl;
    // ROS_INFO("Effective_Sample_Size_: %f", Effective_Sample_Size);

    // --- 2. リサンプリング判定 ---
    if (Effective_Sample_Size < particles_.size() * 0.5)
    {
        std::vector<potbot_lib::DiffDriveAgent> particles_tmp = particles_;

        // --- 3. Systematic Resampling ---
        static std::default_random_engine eng(std::random_device{}());
        double step = 1.0 / particles_.size();
        std::uniform_real_distribution<double> dist(0.0, step);
        double r = dist(eng);

        int weight_num = 0;
        for (size_t step_num = 0; step_num < particles_.size(); ++step_num)
        {
            double u = r + step_num * step;
            while (u > step_sum_weight_[weight_num]) weight_num++;
            particles_[step_num] = particles_tmp[weight_num];
        }

        // --- 4. ノイズ付与（全パーティクル） ---
        double linear_var = 0.01;
        double angular_var = 0.01;
        std::normal_distribution<double> dist_linear(0.0, sqrt(linear_var));
        std::normal_distribution<double> dist_angular(0.0, sqrt(angular_var));

        for (size_t j = 0; j < particles_.size(); ++j) // j=0 から全粒子
        {
            particles_[j].x += dist_linear(eng);
            particles_[j].y += dist_linear(eng);
            particles_[j].yaw += dist_angular(eng);
        }
    }

    // --- 5. 重みを均等に初期化 ---
    std::fill(Likelihood_.begin(), Likelihood_.end(), 1.0 / particles_.size());
}
//リサンプリング(赤井先生ver)
// void PFVisualization::getResamplingRobotPose2(std::vector<double>& step_sum_weight_)
// {
//     step_sum_weight_.clear();
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    PFVisualization pfv;
    // while (ros::ok())
    // {
    //     pfv.resampling();
    //     ros::spinOnce();
    // }
    
    ros::spin();

    return 0;
}