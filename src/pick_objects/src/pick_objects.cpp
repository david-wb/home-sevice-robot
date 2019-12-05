#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void publishStatus(ros::Publisher& status_pub, int status) {
    std_msgs::Int8 msg;
    msg.data = status;
    ROS_INFO("Publishing status %d", msg.data);
    status_pub.publish(msg);
    ros::spinOnce();
}

actionlib::SimpleClientGoalState moveTo(MoveBaseClient& ac, double x, double y) {
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal x=%f y = %f", x, y);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();
  return ac.getState();
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
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

  ros::Publisher status_pub = n.advertise<std_msgs::Int8>("robot_status", 1);
  publishStatus(status_pub, 0);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  publishStatus(status_pub, 0);
  if(moveTo(ac, pickup_x, pickup_y) == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the pickup was reached.");
    publishStatus(status_pub, 1);

    // wait 5 seconds to simulate pickup
    ros::Duration(5.0).sleep();

    publishStatus(status_pub, 2);
    if (moveTo(ac, dropoff_x, dropoff_y) == actionlib::SimpleClientGoalState::SUCCEEDED) {
      publishStatus(status_pub, 3);
      ROS_INFO("Hooray, the dropoff was reached.");
    } else {
      ROS_ERROR("Failed to move to the drop-off pose.");
      exit(1);
    }
  }
  else {
    ROS_ERROR("The base failed to move to the pickup pose.");
    exit(1);
  }

  return 0;
}