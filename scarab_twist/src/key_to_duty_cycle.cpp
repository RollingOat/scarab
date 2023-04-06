#include <iostream>
#include <ros/ros.h>
#include <curses.h>
#include <roboclaw/duty_cycle.h>


using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_to_duty_cycle");
  ros::NodeHandle nh, pnh("~");
  ros::Publisher pub = nh.advertise<roboclaw::duty_cycle>("duty_cycle", 2);

  int inc;
  pnh.param("inc", inc, 100);

  initscr();
  clear();
  noecho();
  cbreak();
  timeout(50); // wait 50 ms
  nodelay(stdscr, TRUE);

  int row = 0;
  mvprintw(row++, 0, "Publishing to %s", pub.getTopic().c_str());
  mvprintw(row++, 0, "a, w, s, d to move and space to halt");

  ros::Rate r(10);
  int max_duty_cycle = 32767;

  int ch;
  roboclaw::duty_cycle duty;
  while (nh.ok()) {
    row = 2;
    mvprintw(row++, 0, "left duty:  %5d", duty.duty_cycle_l);
    mvprintw(row++, 0, "right duty: %5d", duty.duty_cycle_r);
    mvprintw(row++, 0, "");
    refresh();

    ch = getch();

    if (ch != ERR) {
      if (ch == 'w') {
        duty.duty_cycle_l += inc;
        duty.duty_cycle_r += inc;
      } else if (ch == 's') {
        duty.duty_cycle_l -= inc;
        duty.duty_cycle_r -= inc;
      } else if (ch == 'a') {
        duty.duty_cycle_r += inc;
        duty.duty_cycle_l -= inc;
      } else if (ch == 'd') {
        duty.duty_cycle_l += inc;
        duty.duty_cycle_r -= inc;
      } else if (ch == ' ') {
        duty.duty_cycle_l = 0;
        duty.duty_cycle_r = 0;
      }
    }

    if (abs(duty.duty_cycle_r) > max_duty_cycle)
      duty.duty_cycle_r = copysign(max_duty_cycle, duty.duty_cycle_r);
    if (abs(duty.duty_cycle_l) > max_duty_cycle)
      duty.duty_cycle_l = copysign(max_duty_cycle, duty.duty_cycle_l);

    pub.publish(duty);
    ros::spinOnce();
    r.sleep();
  }
  endwin();

  return 0;
}
