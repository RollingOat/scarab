#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <yaml-cpp/yaml.h>

// write a callback function to receive nav_msgs/Odometry as input, publish tf
// to map frame a output
// void objectOdomTFCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
//   ROS_INFO_STREAM("ENTERED CALLBACK FUNCTION");
//   tf2_ros::TransformBroadcaster tf_broadcaster;
//   geometry_msgs::TransformStamped transform_stamped;
//   transform_stamped.header = odom_msg->header;
//   transform_stamped.header.frame_id =
//       "map"; // Set the appropriate parent frame ID
//   transform_stamped.child_frame_id =
//       odom_msg->header.frame_id; // Set the appropriate child frame ID
//   transform_stamped.transform.translation.x = odom_msg->pose.pose.position.x;
//   transform_stamped.transform.translation.y = odom_msg->pose.pose.position.y;
//   transform_stamped.transform.translation.z = odom_msg->pose.pose.position.z;
//   transform_stamped.transform.rotation = odom_msg->pose.pose.orientation;
//   tf_broadcaster.sendTransform(transform_stamped);
// }

class ObjectMover {
private:
  double speed_;
  std::vector<double> start_;
  std::vector<double> end_;
  std::string object_name_;
  ros::NodeHandle nh_;
  int init_vel_sign = 1;
  ros::Publisher cmd_pub_;
  ros::Subscriber odom_sub_;
  bool start_flag_ = false;
  char direction_;

public:
  ObjectMover(ros::NodeHandle &nh, std::string obs_name,
              std::vector<double> &start, std::vector<double> &end,
              double speed)
      : nh_(nh) {
    object_name_ = obs_name;
    start_ = start;
    end_ = end;
    speed_ = speed;
    std::string cmd_name = object_name_ + "_cmd_vel";
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_name, 1);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        object_name_ + "_odom", 100, &ObjectMover::objectOdomCb, this);
    if(start[0] == end[0]){
        direction_ = 'y';
        if(start[1] > end[1]){
            init_vel_sign = -1;
        }
        else{
            init_vel_sign = 1;
        }
    }
    else{
        direction_ = 'x';
        if(start[0] > end[0]){
            init_vel_sign = -1;
        }
        else{
            init_vel_sign = 1;
        }
    }
    
  }
  void objectOdomCb(const nav_msgs::Odometry::ConstPtr &odom_msg){
    double cur_x = odom_msg->pose.pose.position.x;
    double cur_y = odom_msg->pose.pose.position.y;
    double cur_v = odom_msg->twist.twist.linear.x;
    ROS_DEBUG_STREAM("cur_x: " << cur_x << " cur_y: " << cur_y << " cur_v: " << cur_v);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    if(!start_flag_){
        cmd_vel.linear.x = speed_;
        cmd_pub_.publish(cmd_vel);
        start_flag_ = true;
        return;
    }
    if(direction_ == 'x'){
        if(cur_x == end_[0] && cur_v > 0){
            cmd_vel.linear.x = -speed_;
            cmd_pub_.publish(cmd_vel);
            return;
        }
        else if(cur_x == start_[0] && cur_v < 0){
            cmd_vel.linear.x = speed_;
            cmd_pub_.publish(cmd_vel);
            return;
        }
    }
    else{
        if(cur_y == end_[1] && cur_v > 0){
            cmd_vel.linear.x = -speed_;
            cmd_pub_.publish(cmd_vel);
            return;
        }
        else if(cur_y == start_[1] && cur_v < 0){
            cmd_vel.linear.x = speed_;
            cmd_pub_.publish(cmd_vel);
            return;
        }
    }
  }
};

int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "moving_object_node");

  // Create a node handle
  ros::NodeHandle nh;

  // create parameter from yaml file
  std::string yaml_path =
      "/home/jiuzl/scarab_ws/src/scarab/dynamic_logistics_warehouse/config/"
      "dynamic_obstacle.yaml";
  ROS_DEBUG_STREAM("yaml_path: " << yaml_path);
  YAML::Node yaml_node = YAML::LoadFile(yaml_path);
  int num_obstacles = yaml_node["number_of_obstacles"].as<int>();
  std::vector<ObjectMover> ObjectMoverVector;
  for (int i = 0; i < num_obstacles; i++) {
    std::string obs_name = yaml_node["obs" + std::to_string(i+1)]["name"]
                               .as<std::string>(); // obstacle name
    std::vector<double> start = yaml_node["obs" + std::to_string(i+1)]
                                    ["start points"]
                                        .as<std::vector<double>>(); // start
    std::vector<double> end = yaml_node["obs" + std::to_string(i+1)]["end points"]
                                  .as<std::vector<double>>(); // end
    double speed = yaml_node["obs" + std::to_string(i+1)]["speed"]
                       .as<double>(); // speed
    ObjectMoverVector.emplace_back(nh, obs_name, start, end, speed);
  }

  ros::spin();
  return 0;
}
