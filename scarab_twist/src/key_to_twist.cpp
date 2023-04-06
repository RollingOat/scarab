#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <curses.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_to_twist");
  ros::NodeHandle nh, pnh("~");
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  double v_speed, v_inc, w_speed, w_inc;
  pnh.param("vmax", v_speed, 0.8);
  pnh.param("wmax", w_speed, 1.0);
  pnh.param("vinc", v_inc, 0.1);
  pnh.param("winc", w_inc, 0.2);

  initscr();
  clear();
  noecho();
  cbreak();
  timeout(50); // wait 50 ms
  nodelay(stdscr, TRUE);

  int row = 0;
  mvprintw(row++, 0, "Publishing to %s", cmd_pub.getTopic().c_str());
  mvprintw(row++, 0, "Use w, a, s, d to drive, space to halt, and q to quit");

  int ch;
  ros::Rate r(10);
  geometry_msgs::Twist twist;
  while (nh.ok()) {
    row = 2;
    mvprintw(row++, 0, "Angular: % 0.2f", twist.angular.z);
    mvprintw(row++, 0, "Linear:  % 0.2f", twist.linear.x);
    mvprintw(row++, 0, "");
    refresh();

    ch = getch();

    if (ch != ERR) {
      if (ch == 'w') {
        twist.linear.x += v_inc;
      } else if (ch == 's') {
        twist.linear.x -= v_inc;
      } else if (ch == 'a') {
        twist.angular.z += w_inc;
      } else if (ch == 'd') {
        twist.angular.z -= w_inc;
      } else if (ch == ' ') {
        twist.angular.z = 0;
        twist.linear.x = 0;
      } else if (ch == 'q') {
        twist.angular.z = 0;
        twist.linear.x = 0;
        cmd_pub.publish(twist);
        ros::Duration(0.5).sleep();
        break;
      }
    }
    twist.linear.x = min(max(twist.linear.x, -v_speed), v_speed);
    twist.angular.z = min(max(twist.angular.z, -w_speed), w_speed);

    cmd_pub.publish(twist);
    ros::spinOnce();
    r.sleep();
  }
  endwin();

  return 0;
}
