#pragma once
#include <ros/ros.h>
#include "tracker.h"
#include <tf/transform_listener.h>

class TabletopTrackerROS : public TabletopTracker {
public:
  tf::TransformListener listener;
  ros::Publisher cloud_pub;
  ros::Subscriber cloud_sub;
  bool hasPendingMessage;

  TabletopTrackerROS(ros::NodeHandle n);
  void updateTransform();


  void callback(const sensor_msgs::PointCloud2& msg);
  void publish();

};
