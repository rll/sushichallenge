#include "tracker_ros.h"
#include <ros/ros.h>
#include <iostream>
#include "spinning_table_config.h"
using namespace std;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "spinning_tabletop_tracker");
  Parser parser;
  parser.addGroup(SpinConfig());
  parser.read(argc, argv);

  ros::NodeHandle nh;
  TabletopTrackerROS tracker(nh);

  ros::Duration(1).sleep();

  while (ros::ok()) {
    for (int i=0; i < 10; i++) ros::spinOnce();
    if (tracker.hasPendingMessage) {
      ROS_INFO("tracker has pending message. updating");
      try {
	tracker.updateAll();
	tracker.publish();
	tracker.hasPendingMessage = false;
      }
      catch (std::runtime_error err) {
	ROS_ERROR_STREAM("error while updating tracker: " <<err.what());
	ros::Duration(.1).sleep();
      }
    }
    else {
      ros::Duration(0.01).sleep();
    }
    
  }
}
