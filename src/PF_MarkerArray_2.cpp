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
  size_t id = 0;
  while (1)
  {
    std::string param_name = "marker" + std::to_string(id);
    if(!n.hasParam(param_name + "/x")) break;

    double x, y, yaw_deg;
    n.getParam(param_name + "/x", x);
    n.getParam(param_name + "/y", y);
    n.getParam(param_name + "/yaw", yaw_deg);

    marker_pose.push_back(potbot_lib::utility::get_pose(x, y, 0, 0, 0, yaw_deg / 180.0 * M_PI));
    id++;
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    visualization_msgs::MarkerArray marker;
    marker.markers.resize(marker_pose.size());  // ← YAMLで読み込んだ数に応じて確保

    for (size_t i = 0; i < marker_pose.size(); i++)
    {
      marker.markers[i].header.frame_id = "map";
      marker.markers[i].header.stamp = ros::Time::now();
      marker.markers[i].ns = "basic_shapes";
      marker.markers[i].id = i;

      marker.markers[i].type = visualization_msgs::Marker::CUBE;
      marker.markers[i].action = visualization_msgs::Marker::ADD;
      marker.markers[i].lifetime = ros::Duration();

      marker.markers[i].scale.x = 0.196;
      marker.markers[i].scale.y = 0.196;
      marker.markers[i].scale.z = 0.395;

      marker.markers[i].pose = marker_pose[i];
      marker.markers[i].color = potbot_lib::color::get_msg(potbot_lib::color::LIGHT_BLUE);
    }

    marker_pub.publish(marker);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}