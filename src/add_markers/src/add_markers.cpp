#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>

class AddMarkers
{
public:
  AddMarkers(double pickup_x, double pickup_y, double dropoff_x, double dropoff_y);
private:
  double pickup_x;
  double pickup_y;
  double dropoff_x;
  double dropoff_y;
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber robot_status_sub;
  void robotStatusCallback(const std_msgs::Int8::ConstPtr& robotStatus);
  void publishMarker(double x, double y);
  void deleteMarker();
};

AddMarkers::AddMarkers(double pickup_x, double pickup_y, double dropoff_x, double dropoff_y)
{
  this->pickup_x = pickup_x;
  this->pickup_y = pickup_y;
  this->dropoff_x = dropoff_x;
  this->dropoff_y = dropoff_y;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  publishMarker(pickup_x, pickup_y);
  ROS_INFO("published marker");
  ROS_INFO("subscribing to robot_status");
  robot_status_sub = n.subscribe("/robot_status", 1, &AddMarkers::robotStatusCallback, this);
}

void AddMarkers::robotStatusCallback(const std_msgs::Int8::ConstPtr& robotStatus)
{
  ROS_INFO("Received robot status %d", robotStatus->data);

  switch (robotStatus->data) {
      case 0: ROS_INFO("Going to pickup"); publishMarker(pickup_x, pickup_y); break;
      case 1: ROS_INFO("At pickup"); deleteMarker(); break;
      case 2: ROS_INFO("Going to dropoff"); break;
      case 3: ROS_INFO("At dropoff"); publishMarker(dropoff_x, dropoff_y); break;
      default: ROS_INFO("Unknown robot status");
  }
}

void AddMarkers::deleteMarker() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETE;
    
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
}

void AddMarkers::publishMarker(double x, double y) {
  ROS_INFO("Publishing marker at (%f, %f)", x, y);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      exit(1);
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
}

int main( int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  double pickup_x;
  double pickup_y;
  double dropoff_x;
  double dropoff_y;

  if (!n.getParam("/home_service/pickup_x", pickup_x))
    ROS_ERROR("Failed to get param 'pickup_x'");
  if (!n.getParam("/home_service/pickup_y", pickup_y))
    ROS_ERROR("Failed to get param 'pickup_y'");
  if (!n.getParam("/home_service/dropoff_x", dropoff_x))
    ROS_ERROR("Failed to get param 'dropoff_x'");
  if (!n.getParam("/home_service/dropoff_y", dropoff_y))
    ROS_ERROR("Failed to get param 'dropoff_y'");

  AddMarkers addMarkers(pickup_x, pickup_y, dropoff_x, dropoff_y);
  ROS_INFO("ros spin");
  ros::spin();
}
