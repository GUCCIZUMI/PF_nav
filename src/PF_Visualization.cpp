#include <PF_nav/PF_Visualization.h>

std::ofstream Likelihood_txt("/home/ros/catkin_ws/user/src/data/simulator/Likelihood.txt");
std::ofstream Ess_txt("/home/ros/catkin_ws/user/src/data/simulator/Ess.txt");
std::ofstream Estimate_position("/home/ros/catkin_ws/user/src/data/simulator/Estimate_position.csv");
std::ofstream Robot_command("/home/ros/catkin_ws/user/src/data/simulator/Robot_command.csv");
std::ofstream Pt_Position("/home/ros/catkin_ws/user/src/data/simulator/Pt_position.csv");

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
	// pub_particles_state_ = nh.advertise<potbot_msgs::ObstacleArray>("particles_state", 1);

	// ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,inipose_callback);

    // pub_odom_encoder_ = nh.advertise<geometry_msgs::PoseArray>("odom/encoder", 1); //2024年11月14日追加

    pub_estimated_robot_ = nh.advertise<nav_msgs::Odometry>("odom/estimated", 1);

    // サブスクライバの作成
    sub_marker_ = nh.subscribe("marker", 1000, &PFVisualization::markerCallback,this);
    sub_robot_pose_ = nh.subscribe("odom", 1000, &PFVisualization::robotPoseCallback,this);
    sub_robot_command_ = nh.subscribe("odom/truth", 1000, &PFVisualization::robotCommandCallback,this);


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

void PFVisualization::localization()
{
    // if (initial_Time) {
    //     initialparticlepose();
    // }

    filteringdecision();
    
    if(subscribed_robot_command){
        
        Count_step_ = Count_step_ + 1;
        Likelihood_txt <<  " 実行回数 "  << Count_step_ << std :: endl;
        Pt_Position <<  " 実行回数 "  << Count_step_ <<  " 実行回数 "  << Count_step_ <<  " 実行回数 "  << Count_step_ << std :: endl;

        updateParticles();

        std::vector<int> in_range_ids;
        getObservedLandmark(in_range_ids);
        initLiklihood();
    
        for (const auto& marker_id:in_range_ids)
        {
            getLikelihood(marker_id);
        }
    
        normLiklihood();

        getEstimatedRobotPose();

        getResamplingRobotPose1(step_sum_weight_);
    }else{
        std::cout << "not robot command" << std::endl;
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

    for (size_t i = 1; i < particles_.size(); ++i)
	{
        particles_[i].x = particles_[i].x + initial_particle_Position_noise(generator);
        particles_[i].y = particles_[i].y + initial_particle_Position_noise(generator);
        particles_[i].yaw = particles_[i].yaw + initial_particle_yaw_noise(generator);
	}

    initial_Time = false;    
}

void PFVisualization::updateParticles()
{
    geometry_msgs::PoseArray particles_msg;
    for (auto& p:particles_)
    {
        double v = odom_msg_.twist.twist.linear.x;
        double omega = odom_msg_.twist.twist.angular.z;
        p.v = v;
        p.omega = omega;
        p.deltatime = 1.0/50.0;
        p.update();

        nav_msgs::Odometry p_msg;
        potbot_lib::utility::to_msg(p, p_msg); //p.to_msg(p_msg);
        particles_msg.poses.push_back(p_msg.pose.pose);
    }
    
    particles_msg.header.frame_id = "map";
    particles_msg.header.stamp = ros::Time::now();

    pub_particles_.publish(particles_msg);

    // //2024年11月14日追加事項(エンコーダーによる観測)---------------------------------
    // nav_msgs::Odometry encoder_msg;

    // encoder_msg.v = odom_msg_.twist.twist.linear.x;
    // encoder_msg.omega = odom_msg_.twist.twist.angular.z;
    // encoder_msg.deltatime = 1.0/50.0;
    // encoder_msg.update();

    // pub_odom_encoder_.publish(encoder_msg);
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

//範囲内マーカー観測プロセス
void PFVisualization::getObservedLandmark(std::vector<int>& in_range)
{
    ros::NodeHandle n("~");

    double norm_noise_mean_Scan_distance = 0;
	double norm_noise_variance_Scan_distance = 0.1;
	double norm_noise_mean_Scan_angle = 0;
	double norm_noise_variance_Scan_angle = 0.1;
    double norm_noise_mean_Scan_Long_distance = 0; //遠距離(5m < d < 8m)における観測ノイズ(平均)
    double norm_noise_variance_Scan_Long_distance = 0; //遠距離(5m < d < 8m)における観測ノイズ(分散)

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

    in_range.clear();

    for(size_t i = 0; i < marker_positions_.size(); ++i)
    {
        const auto& marker_pose = marker_positions_[i];
        double dx = marker_pose.position.x - robot_pose_x_; //マーカーとロボットのx座標距離(真値の距離：センサ値)
        double dy = marker_pose.position.y - robot_pose_y_; //マーカーとロボットのy座標距離(真値の距離：センサ値)
        Robot_distance_ = std::sqrt(dx * dx + dy * dy); //マーカーとロボットの直線距離(真値の距離：センサ値)
        robot_distances_.push_back(Robot_distance_);

        if(Robot_distance_ <= 5.0){
            Scan_distance_ = Robot_distance_ + distribution_Scan_distance(generator); //マーカーとロボットの直線距離(真値の距離(近距離)：ノイズ入りセンサ値)
        }else if(5.0 < Robot_distance_ < 8.0 ){
            Scan_distance_ = Robot_distance_ + distribution_Scan_Long_distance(generator); //マーカーとロボットの直線距離(真値の距離(遠距離)：ノイズ入りセンサ値)
        }else{
            Scan_distance_ = 0; //マーカーとロボットの直線距離(真値の距離(それ以外)：ノイズ入りセンサ値)
        }

        robot_scan_distances_.push_back(Scan_distance_);

        if(Robot_distance_ > radius_){
            continue;
        }
    
        Robot_angle_ = robot_pose_yaw_ - std::atan2(dy, dx); //マーカーとロボットの角度(真値の角度：センサ値)
        robot_angles_.push_back(Robot_angle_);

        Scan_angle_ = Robot_angle_ + distribution_Scan_angle(generator); //マーカーとロボットの角度(真値の角度：ノイズ入りセンサ値)
        robot_scan_angles_.push_back(Scan_angle_);

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

    robot_distances_.clear();
    robot_scan_distances_.clear();
    robot_angles_.clear();
    robot_scan_angles_.clear();
}

//マーカー、パーティクル間誤差、従来法による尤度計算(距離、角度)
void PFVisualization::getLikelihood(size_t marker_id)
{
    const auto& marker = marker_positions_[marker_id];
    double Scan_distance_ = robot_scan_distances_[marker_id];
    double Scan_angle_ = robot_scan_angles_[marker_id];
    

    for (size_t j = 0; j < particles_.size(); ++j)
    {
        const auto & particle = particles_[j]; 

        dis_X_ = marker.position.x - particle.x; //マーカとパーティクルのx座標距離(予測値の距離：推定値)
        dis_Y_ = marker.position.y - particle.y; //マーカとパーティクルのy座標距離(予測値の距離：推定値)
        
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_); //マーカとパーティクルの直線距離
        double particle_angle = particle.yaw - atan2(dis_Y_ , dis_X_); //マーカとパーティクルの角度
        if(particle_angle < 0){
            particle_angle += 2 * M_PI;
        }

        // dis_var_ = dis_var_ * dis_X_ * dis_X_;  //尤度関数分散値の変更式(実機の方に実装されている分散はこっち)

        // double w_dis = 1/(sqrt(2 * M_PI * dis_var_))*exp(-((abs(Scan_distance_)-abs(particle_distance))*(abs(Scan_distance_)-abs(particle_distance)))/(2*dis_var_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ - (- particle_angle - particle.yaw)) * ( Scan_angle_ - (- particle_angle - particle.yaw))) / (2 * ang_var_))+1e-100;
        
        if(Scan_angle_ * particle_angle > 0 && particle_angle > 1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI))) * ( Scan_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI)))) / (2  * ang_var_))+1e-100;
        }else if (Robot_angle_ * particle_angle > 0 && particle_angle < -1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI))) * ( Scan_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI)))) / (2  * ang_var_))+1e-100;
        }
        

        // double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_ang_log);

        Likelihood_[j]*=weight;
    }
}   

//マーカー、パーティクル間誤差、提案法による尤度計算(距離、角度)
void PFVisualization::getLikelihood_main(size_t marker_id)
{
    const auto& marker = marker_positions_[marker_id];
    double Scan_distance_ = robot_scan_distances_[marker_id];
    double Scan_angle_ = robot_scan_angles_[marker_id];
    

    for (size_t j = 0; j < particles_.size(); ++j)
    {
        const auto & particle = particles_[j]; 

        dis_X_ = marker.position.x - particle.x; //マーカとパーティクルのx座標距離(予測値の距離：推定値)
        dis_Y_ = marker.position.y - particle.y; //マーカとパーティクルのy座標距離(予測値の距離：推定値)
        
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_);
        double particle_angle = particle.yaw - atan2(dis_Y_ , dis_X_);
        
        if(particle_angle < 0){
            particle_angle += 2 * M_PI;
        }

        dis_var_ = dis_var_ * dis_X_ * dis_X_;  //尤度関数分散値の変更式(実機の方に実装されている分散はこっち)
        
        //変曲点に着目した距離分散変動
        observe_scan_distance_error_ = abs(abs(Scan_distance_)-abs(particle_distance));
        ROS_INFO_STREAM("observe_scan_distance_error_: " << observe_scan_distance_error_ <<
                   " Scan_distance_: " << Scan_distance_ <<
                   " particle_distance: " << particle_distance);

        if (abs(abs(Scan_distance_)-abs(particle_distance))>0.8&&abs(abs(Scan_distance_)-abs(particle_distance))<1.3)
        {
            dis_var_=abs(abs(Scan_distance_)-abs(particle_distance))*abs(abs(Scan_distance_)-abs(particle_distance));
            Scan_distance_ = Scan_distance_ - 0.80;
        }

        // double w_dis = 1/(sqrt(2 * M_PI * dis_var_))*exp(-((abs(Scan_distance_)-abs(particle_distance))*(abs(Scan_distance_)-abs(particle_distance)))/(2*dis_var_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ - (- particle_angle - particle.yaw)) * ( Scan_angle_ - (- particle_angle - particle.yaw))) / (2 * ang_var_))+1e-100;

        // if (abs(abs(Scan_distance_)-abs(particle_distance))>1.3)
        // {
        //     w_dis=1;
        // }
        
        if(Scan_angle_ * particle_angle > 0 && particle_angle > 1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI))) * ( Scan_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI)))) / (2  * ang_var_))+1e-100;
        }else if (Robot_angle_ * particle_angle > 0 && particle_angle < -1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Scan_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI))) * ( Scan_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI)))) / (2  * ang_var_))+1e-100;
        }
        

        // double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_ang_log);

        Likelihood_[j]*=weight;
                
    }
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
        Likelihood_txt <<  " 尤度 " <<  "  "  << Likelihood_[j] << std::endl;     
        
        Pt_Position << "Pt_X" << " " << particle.x << " " << "Pt_Y" << " " << particle.y << " " << "Pt_Yaw" << " "  << particle.yaw << " " << std::endl;

        Particle_Est_RobotX += particle.x * w;
        Particle_Est_RobotY += particle.y * w;
        Particle_Est_RobotYaw += particle.yaw * w;

    }

    Estimate_position << "EstX" << " " << Particle_Est_RobotX << " " << "EstY" << " " << Particle_Est_RobotY << " " << "EstTh" << " " << Particle_Est_RobotYaw << " " << std :: endl;
    
    // std::cout << "=========" << std::endl;
    // std::cout << "Particle_Est_RobotX=" <<Particle_Est_RobotX<< std::endl;
    // std::cout << "Particle_Est_RobotY=" <<Particle_Est_RobotY<< std::endl;
    // std::cout << "Particle_Est_RobotTH=" <<Particle_Est_RobotYaw<< std::endl;
    // ROS_INFO_STREAM("Norm total weight: " << Norm_total_weight <<
    //                 " EstX: " << Particle_Est_RobotX <<
    //                 " EstY: " << Particle_Est_RobotY <<
    //                 " EstTh: " << Particle_Est_RobotYaw);

    nav_msgs::Odometry est_msg;
    est_msg.header.stamp = ros::Time::now();
    est_msg.header.frame_id = odom_msg_.header.frame_id;
    est_msg.child_frame_id = odom_msg_.child_frame_id;
    est_msg.pose.pose = potbot_lib::utility::get_pose(Particle_Est_RobotX,Particle_Est_RobotY,0,0,0,Particle_Est_RobotYaw);
    pub_estimated_robot_.publish(est_msg);
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
    step_sum_weight_.clear();
    double step_weight = 0.0;
    double Effective_Sample_Size = 0.0;
    double ESS_sum = 0.0;
    
    for ( size_t i = 0; i < particles_.size(); ++i)
    {
       step_weight += Likelihood_[i];
       ESS_sum += Likelihood_[i] * Likelihood_[i];
       step_sum_weight_.push_back(step_weight);
    }
    
    Effective_Sample_Size = 1 / ESS_sum;

    Ess_txt << "Effective_Sample_Size_" <<  "  "  << Effective_Sample_Size << std::endl;
    // ROS_INFO("Effective_Sample_Size_: %f", Effective_Sample_Size);

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(0,step_sum_weight_[ particles_.size() - 1] / particles_.size());
    double darts = distr(eng);
    darts = 0;

    int weight_num = 0;
    int step_num = 0;

    std::vector<potbot_lib::DiffDriveAgent> particles_tmp = particles_;
    
     
    if ( Effective_Sample_Size < particles_.size() * 0.5)
    {
         ROS_INFO("Not Active Resampling");
    }else
    {
        ROS_INFO("IN Active Resampling");
        while(step_num <  particles_.size())
        {
            if(darts < step_sum_weight_[weight_num])
            {

                particles_[step_num].x = particles_tmp[weight_num].x;
                particles_[step_num].y = particles_tmp[weight_num].y;
                particles_[step_num].yaw = particles_tmp[weight_num].yaw;

                // darts += (step_sum_weight_[ particles_.size() - 1] / particles_.size());
                darts += 0.001;
                step_num += 1;
            }else
            {
                weight_num += 1;   
            }
        }
        // double second_noise_mean_linear_velocity = 0;
	    // double second_noise_variance_linear_velocity = 0.0001;
	    // double second_noise_mean_angular_velocity = 0;
	    // double second_noise_variance_angular_velocity = 0.0001;

        // std::random_device rd3;
        // std::default_random_engine generator(rd3());
        // std::normal_distribution<double> distribution_linear_velocity_2(second_noise_mean_linear_velocity, sqrt(second_noise_variance_linear_velocity));
	    // std::normal_distribution<double> distribution_angular_velocity_2(second_noise_mean_angular_velocity, sqrt(second_noise_variance_angular_velocity));

        // for (size_t j = 1; j < particles_.size(); ++j)
	    // {
        //     particles_[j].x = particles_[j].x + distribution_linear_velocity_2(generator);
        //     particles_[j].y = particles_[j].x + distribution_linear_velocity_2(generator);
        //     particles_[j].yaw = particles_[j].x + distribution_angular_velocity_2(generator);
	    // }
    }
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