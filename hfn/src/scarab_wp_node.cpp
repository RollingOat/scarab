#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <scarab_msgs/MoveAction.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

#include <random>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scarab_wp");
  ros::NodeHandle nh = ros::NodeHandle("~");

  // Get params
  std::vector<double> mapmin, mapmax;
  nh.getParam("mapmin", mapmin);
  nh.getParam("mapmax", mapmax);

  double goal_timeout;
  nh.param("goal_timeout", goal_timeout, 10.0);

  ROS_INFO("Map bounds x: %g %g y: %g %g ", mapmin[0], mapmax[0], mapmin[1], mapmax[1]);

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist_x(mapmin[0], mapmax[0]);
  std::uniform_real_distribution<double> dist_y(mapmin[1], mapmax[1]);
  std::uniform_real_distribution<double> dist_yaw(0, 3.14);

  actionlib::SimpleActionClient<scarab_msgs::MoveAction> ac("move", true);

  ROS_INFO("Waiting for goal action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending random goal.");

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;

    msg.pose.position.x = dist_x(mt);
    msg.pose.position.y = dist_y(mt);
    double goal_yaw = dist_yaw(mt);
    tf::Quaternion q_rot = tf::createQuaternionFromRPY(0.0, 0.0, goal_yaw);
    msg.pose.orientation.x = q_rot.x();
    msg.pose.orientation.y = q_rot.y();
    msg.pose.orientation.z = q_rot.z();
    msg.pose.orientation.w = q_rot.w();

    msg.header.stamp = ros::Time::now();

    ROS_INFO("Goal x: %g y: %g yaw: %g", msg.pose.position.x, msg.pose.position.y, goal_yaw);

    // send a goal to the action
    scarab_msgs::MoveGoal goal;

    goal.target_poses.push_back(msg);

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(goal_timeout));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out of %g secs", goal_timeout);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

