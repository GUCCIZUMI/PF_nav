#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <potbot_lib/utility_ros.h>

#include <string>
#include <math.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_marker_publisher1");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);

  ros::NodeHandle n("~");
  std::vector<geometry_msgs::Pose> marker_pose;
  std::vector<geometry_msgs::Vector3> marker_scale;
  size_t id = 0;
  while (1)
  {
    std::string param_name = "marker" + std::to_string(id);
    if(!n.hasParam(param_name + "/x")) break;
    double x,y,yaw_deg;
    n.getParam(param_name + "/x", x);
    n.getParam(param_name + "/y", y);
    n.getParam(param_name + "/yaw", yaw_deg);
    marker_pose.push_back(potbot_lib::utility::get_pose(x,y,0,0,0,yaw_deg/180.0*M_PI));
    id++;
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    visualization_msgs::MarkerArray marker;
    marker.markers.resize(3);

    marker.markers[0].header.frame_id = "map";
    marker.markers[0].header.stamp = ros::Time::now();
    marker.markers[0].ns = "basic_shapes";
    marker.markers[0].id = 0;

    marker.markers[0].type = visualization_msgs::Marker::CUBE;
    marker.markers[0].action = visualization_msgs::Marker::ADD;
    marker.markers[0].lifetime = ros::Duration();

    //double t = marker.header.stamp.toSec();

    marker.markers[0].scale.x = 0.196;
    marker.markers[0].scale.y = 0.196;
    marker.markers[0].scale.z = 0.395;
    
    marker.markers[0].pose = marker_pose[0];
    marker.markers[0].color = potbot_lib::color::get_msg(potbot_lib::color::LIGHT_BLUE);
    
    marker.markers[1].header.frame_id = "map";
    marker.markers[1].header.stamp = ros::Time::now();
    marker.markers[1].ns = "basic_shapes";
    marker.markers[1].id = 1;

    marker.markers[1].type = visualization_msgs::Marker::CUBE;
    marker.markers[1].action = visualization_msgs::Marker::ADD;
    marker.markers[1].lifetime = ros::Duration();

    //double t = marker.header.stamp.toSec();

    marker.markers[1].scale.x = 0.196;
    marker.markers[1].scale.y = 0.196;
    marker.markers[1].scale.z = 0.395;
    
    marker.markers[1].pose = marker_pose[1];
    marker.markers[1].color = potbot_lib::color::get_msg(potbot_lib::color::LIGHT_BLUE);

    marker.markers[2].header.frame_id = "map";
    marker.markers[2].header.stamp = ros::Time::now();
    marker.markers[2].ns = "basic_shapes";
    marker.markers[2].id = 2;

    marker.markers[2].type = visualization_msgs::Marker::CUBE;
    marker.markers[2].action = visualization_msgs::Marker::ADD;
    marker.markers[2].lifetime = ros::Duration();

    //double t = marker.header.stamp.toSec();

    marker.markers[2].scale.x = 0.196;
    marker.markers[2].scale.y = 0.196;
    marker.markers[2].scale.z = 0.395;
    
    marker.markers[2].pose = marker_pose[2];
    marker.markers[2].color = potbot_lib::color::get_msg(potbot_lib::color::LIGHT_BLUE);
    
    marker_pub.publish(marker);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
