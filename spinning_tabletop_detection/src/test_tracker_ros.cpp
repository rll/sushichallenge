#include "tracker_ros.h"
#include <ros/ros.h>
#include <iostream>
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "spinning_tabletop_tracker");
  ros::NodeHandle nh;
  TabletopTrackerROS tracker(nh);

  ros::Duration(1).sleep();

  ROS_INFO("hi");
  while (true) {
    for (int i=0; i < 10; i++) ros::spinOnce();
    if (tracker.hasPendingMessage) {
      ROS_INFO("got one!");
      tracker.updateAll();
      tracker.publish();
      tracker.hasPendingMessage = false;
    }
    else {
      ros::Duration(0.01).sleep();
    }
    
  }
}
