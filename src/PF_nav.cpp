#include <ros/ros.h>
#include <potbot_lib/utility_ros.h>
#include <potbot_lib/pid.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <random>

std::vector<potbot_lib::controller::PID> g_robot;
std::random_device g_rd;
std::default_random_engine g_generator(g_rd());
double g_norm_noise_mean_initial_particle_X = 0; //初期パーティクルに載せる正規分布平均(初期設定)
double g_norm_noise_mean_initial_particle_Y = 0; //初期パーティクルに載せる正規分布平均(初期設定)
double g_norm_noise_mean_initial_particle_yaw = 0; //初期パーティクルに載せる正規分布平均(初期設定)
double g_norm_noise_variance_initial_particle_X = 0.1; //初期パーティクルに載せる正規分布分散(初期設定)
double g_norm_noise_variance_initial_particle_Y = 0.1; //初期パーティクルに載せる正規分布分散(初期設定)
double g_norm_noise_variance_initial_particle_yaw = 0.1; //初期パーティクルに載せる正規分布分散(初期設定)

void init_particles()
{
	//<--追加要素-->
	std::normal_distribution<double> distribution_initial_particle_X(g_norm_noise_mean_initial_particle_X, sqrt(g_norm_noise_variance_initial_particle_X));
	std::normal_distribution<double> distribution_initial_particle_Y(g_norm_noise_mean_initial_particle_Y, sqrt(g_norm_noise_variance_initial_particle_Y));
	std::normal_distribution<double> distribution_initial_particle_yaw(g_norm_noise_mean_initial_particle_yaw, sqrt(g_norm_noise_variance_initial_particle_yaw));

    
	//<--追加要素(パーティクル初期配置について)-->----------------------------------------------------------------------------------------------------------------------
    for (size_t i = 1; i < g_robot.size(); i++){
		
		nav_msgs::Odometry particle;
		potbot_lib::utility::to_msg(g_robot[i], particle);	// <- g_robot[i].to_msg(particle);
		
		
		particle.pose.pose.position.x += distribution_initial_particle_X(g_generator);
		particle.pose.pose.position.y += distribution_initial_particle_Y(g_generator);
		double yaw = tf2::getYaw(particle.pose.pose.orientation) + distribution_initial_particle_yaw(g_generator);
        particle.pose.pose.orientation = potbot_lib::utility::get_quat(0,0,yaw);

		potbot_lib::utility::to_agent(particle, g_robot[i]) ; // <- g_robot[i].set_msg(particle);
	}
    //---------------------------------------------------------------------------------------------------------------------------------------------------------------
}

//rviz関連のcallback関数----------------------------------------------------------------------------------------------------------------------------------------------
void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	nav_msgs::Odometry ini_pose;
	ini_pose.header.frame_id = "map" ;
	ini_pose.pose = msg.pose;
	for (auto& robo : g_robot) potbot_lib::utility::to_agent(ini_pose, robo); //ランダムでrobotの状態を設定、そのすべてにroboでアクセスオドメトリ等の定義設定
	init_particles();
}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
	g_robot[0].setTargetPoint(
		potbot_lib::Pose{
			msg.pose.position.x,
			msg.pose.position.y,0,0,0,
			tf2::getYaw(msg.pose.orientation)
			}
		);
}

void param_callback(const PF_nav::PF_navConfig& param, uint32_t level)
{
	g_robot[0].setGain(	param.gain_p,
						param.gain_i,
						param.gain_d);

	g_robot[0].setMargin(	param.stop_margin_angle,
							param.stop_margin_distance);

	g_robot[0].setLimit(	param.max_linear_velocity,
							param.max_angular_velocity);
}


// //<--追加要素(マーカーの配置、座標について)>-----------------------------------------------------------------------------------------------------------------------
// 	void Marker_callback(const visualization_msgs::MarkerArray& Marker_msg){
// 		std::vector<geometry_msgs::Point> points = Marker_msg->points;

// 	}

// //---------------------------------------------------------------------------------------------------------------------------------------------------------------




//-----------------------------------------------------------------------------------------------------------------------------------------

//(ここから実際に使用するプログラム----------------------------------------------------------------------------------------------------------
int main(int argc,char **argv){
	ros::init(argc,argv,"controller_tutorial");

	ros::NodeHandle n("~");

	double control_frequency = 50.0; //周期(初期設定)	
	int particle_num = 100; //パーティクル個数(初期設定)
	double norm_noise_mean_linear_velocity = 0; //速度に載せる正規分布平均(初期設定)
	double norm_noise_variance_linear_velocity = 0.1; //速度に載せる正規分布分散(初期設定)
	double norm_noise_mean_angular_velocity = 0; //角速度に載せる正規分布平均(初期設定)
	double norm_noise_variance_angular_velocity = 0.1; //速度に載せる正規分布平均(初期設定)

	n.getParam("control_frequency", control_frequency); //launchファイルから取得周期(50.0)
	n.getParam("particle_num", particle_num); //launchファイルから取得パーティクル個数(1000)
	n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity); //launchファイルから取得速度に載せる正規分布平均(0.0) 
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity); //launchファイルから取得速度に載せる正規分布分散(0.1)
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity); //launchファイルから取得角速度に載せる正規分布平均(0.0)
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity); //launchファイルから速度に載せる正規分布平均(0.1)
	//<--追加要素-->
	n.getParam("norm_noise_mean_initial_particle_X", g_norm_noise_mean_initial_particle_X); //launchファイルから取得初期パーティクル(x座標)に載せる正規分布平均(0.0)変更予定
	n.getParam("norm_noise_variance_initial_particle_X", g_norm_noise_variance_initial_particle_X); //launchファイルから取得初期パーティクル(x座標)に載せる正規分布分散(0.5)
    n.getParam("norm_noise_mean_initial_particle_Y", g_norm_noise_mean_initial_particle_Y); //launchファイルから取得初期パーティクル(y座標)に載せる正規分布平均(0.0)変更予定
	n.getParam("norm_noise_variance_initial_particle_Y", g_norm_noise_variance_initial_particle_Y); //launchファイルから取得初期パーティクル(y座標)に載せる正規分布分散(0.5)
    n.getParam("norm_noise_mean_initial_particle_yaw", g_norm_noise_mean_initial_particle_yaw); //launchファイルから取得初期パーティクル(yaw)に載せる正規分布平均(0.0)変更予定
	n.getParam("norm_noise_variance_initial_particle_yaw", g_norm_noise_variance_initial_particle_yaw); //launchファイルから取得初期パーティクル(yaw)に載せる正規分布分散(3.0 * M_PI / 180.0)

	g_robot.resize(particle_num+1); //grobotの数の定義(ロボット実機の個数も想定するためP_num+1)

	ros::NodeHandle nh;

	ros::Publisher pub_odom					= nh.advertise<nav_msgs::Odometry>("odom", 1); //140行で設定している変数の送信定義(robot_pose)
	ros::Publisher pub_particles			= nh.advertise<geometry_msgs::PoseArray>("particles", 1); //141行で設定している変数の送信定義(particles_msg)
	ros::Publisher pub_particles_state		= nh.advertise<potbot_msgs::ObstacleArray>("particles_state", 1); //142行で設定している変数の送信定義(particle_state_msg)

	ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,inipose_callback); //inipose::初期配置の受信定義(どこから送信しているかはわからない)
	ros::Subscriber sub_goal				= nh.subscribe("move_base_simple/goal",1,goal_callback); //goal::目標地点の受信定義(どこから送信しているかはわからない)

	//ros::Subscriber Marker_Sub              = nh.subscribe("marker_array",1,Marker_callback);

    //ここについても何をしているかわからないので小池さんに聞くこと----------------------------------------------------------------------------------------------------------------
	dynamic_reconfigure::Server<PF_nav::PF_navConfig> server;
	dynamic_reconfigure::Server<PF_nav::PF_navConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);
	//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	ros::Rate rate(control_frequency); //ROSの実行周期(50.0)

	nav_msgs::Odometry robot_pose; //robot_pose定義(odometry型)
	robot_pose.header.frame_id = "map";
	robot_pose.pose.pose = potbot_lib::utility::get_pose(0,0,0,0,0,0);	//ロボット初期位置 x y z r p y

	geometry_msgs::PoseArray particles_msg; //particle_msg定義(PoseArrey型)
	particles_msg.header.frame_id = robot_pose.header.frame_id; //tfに関することなんだろうけど詳しくは小池さんに聞いて(mapに設定してる)
	potbot_msgs::ObstacleArray particle_state_msg; //particle_state_msg定義(ObstacleArrey型)
	particle_state_msg.header.frame_id = robot_pose.header.frame_id; //tfに関することなんだろうけど詳しくは小池さんに聞いて(mapに設定してる)
	
	for (auto& robo : g_robot)
	{
		robo.deltatime = 1.0/control_frequency;;
		potbot_lib::utility::to_agent(robot_pose, robo);
	}

	init_particles();

	for (size_t i = 0; i < particle_num; i++) 
	{
		nav_msgs::Odometry odom;
		potbot_lib::utility::to_msg(g_robot[i+1], odom);
		particles_msg.poses.push_back(odom.pose.pose);
		potbot_msgs::Obstacle p;
		p.pose = odom.pose.pose;
		particle_state_msg.data.push_back(p);
	};

	

    std::normal_distribution<double> distribution_linear_velocity(norm_noise_mean_linear_velocity, sqrt(norm_noise_variance_linear_velocity));
	std::normal_distribution<double> distribution_angular_velocity(norm_noise_mean_angular_velocity, sqrt(norm_noise_variance_angular_velocity));
    

	while (ros::ok())
	{

		g_robot[0].calculateCommand();
		g_robot[0].update();
		
		potbot_lib::utility::to_msg(g_robot[0], robot_pose);

		particles_msg.header = robot_pose.header;
		particle_state_msg.header = robot_pose.header;
		double v_truth = robot_pose.twist.twist.linear.x;
		double omega_truth = robot_pose.twist.twist.angular.z;
		double v_noise = v_truth + distribution_linear_velocity(g_generator); //(変更点::パーティクルに載せる速度は一定)
		double omega_noise = omega_truth + distribution_angular_velocity(g_generator); //(変更点::パーティクルに載せる角速度は一定)

		if (v_truth != 0 || omega_truth != 0)
		{
			
			for (size_t i = 1; i < particle_num+1; i++)
			{
				//double v_noise = v_truth + distribution_linear_velocity(generator);
				//double omega_noise = omega_truth + distribution_angular_velocity(generator);
				nav_msgs::Odometry particle;
				potbot_lib::utility::to_msg(g_robot[i], particle);
				particle.twist.twist.linear.x = v_noise;
				particle.twist.twist.angular.z = omega_noise;
				potbot_lib::utility::to_agent(particle, g_robot[i]);
				g_robot[i].update();

				//観測モデル実装予定(関数定義にするかも)---------------------------------------------------------------------------------------------------------------


				//---------------------------------------------------------------------------------------------------------------------------------------------------


				//自己位置推定過程(関数定義にするかも)-----------------------------------------------------------------------------------------------------------------


				//---------------------------------------------------------------------------------------------------------------------------------------------------

				
				//リサンプリング過程(関数定義にするかも)---------------------------------------------------------------------------------------------------------------
				

				//---------------------------------------------------------------------------------------------------------------------------------------------------
			    

				particles_msg.poses[i-1] = particle.pose.pose;

				particle_state_msg.data[i-1].pose = particle.pose.pose;
				particle_state_msg.data[i-1].twist = particle.twist.twist;

			}
		}

		pub_odom.publish(robot_pose);
		pub_particles.publish(particles_msg);
		pub_particles_state.publish(particle_state_msg);
        
		ros::spinOnce();
		rate.sleep();
		
	}

	return 0;
}