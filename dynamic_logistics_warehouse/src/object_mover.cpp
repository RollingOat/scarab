#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>

int periodic_function(double t, double T){
    int i = (int)floor(t / T);
    if(i % 2 == 0){
        return 1;
    }
    return -1;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "moving_object_node");
    
    // Create a node handle
    ros::NodeHandle nh;
    
    int num_objects = 6;

    // Create publishers
    std::vector<ros::Publisher> pub_vec(num_objects);
    for(int i = 0; i<num_objects; i++){
        std::string cmd_name = "/obj"+std::to_string(i+1)+"_cmd_vel";
        pub_vec[i] = nh.advertise<geometry_msgs::Twist>(cmd_name, 10);
    }

    ros::Time current_time = ros::Time::now();
    double init_time_sec = current_time.toSec();
    double v = 0.5;
    std::vector<std::vector<double>> traj_len = {
        {14, 0},
        {14, 0},
        {14, 0},
        {0, 20},
        {0, 6},
        {0, 20}
    };

    while(ros::ok()){
        ros::Time current_time = ros::Time::now();
        double current_time_sec = current_time.toSec();
        double t = current_time_sec - init_time_sec;
        for(int i = 0; i<num_objects; i++){
            double T_x = traj_len[i][0]/v;
            double T_y = traj_len[i][1]/v;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = traj_len[i][0] == 0? 0:v * periodic_function(t, T_x);
            cmd_vel.linear.y = traj_len[i][1] == 0? 0:v * periodic_function(t, T_y);
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0;
            pub_vec[i].publish(cmd_vel);
        }
    }
    return 0;
}
