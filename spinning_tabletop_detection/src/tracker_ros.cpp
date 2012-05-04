#include "tracker_ros.h"
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
using namespace std;

Eigen::Affine3f toEigenTransform(const btTransform& transform) {
  btVector3 transBullet = transform.getOrigin();
  btQuaternion quatBullet = transform.getRotation();
  Eigen::Translation3f transEig;
  transEig = Eigen::Translation3f(transBullet.x(), transBullet.y(), transBullet.z());
  Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
  Eigen::Affine3f out = transEig*rotEig;
  return out;
}

TabletopTrackerROS::TabletopTrackerROS(ros::NodeHandle nh) : hasPendingMessage(false) {
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("spinning_tabletop/clouds",100);
  cloud_sub = nh.subscribe("/qwe",1,&TabletopTrackerROS::callback, this);
}

void TabletopTrackerROS::callback(const sensor_msgs::PointCloud2& msg) {
  ColorCloudPtr cloud(new ColorCloud());
  pcl::fromROSMsg(msg, *cloud);
  setLatest(cloud);
  hasPendingMessage = true;
}

void TabletopTrackerROS::updateTransform() {
  tf::StampedTransform stamped_transform;
  listener.lookupTransform("base_footprint", "/openni_rgb_optical_frame", ros::Time(0), stamped_transform);
  transform = toEigenTransform(stamped_transform.asBt());

}

ColorCloudPtr addColor(ColorCloudPtr in, uint8_t r, uint8_t g, uint8_t b) {
  ColorCloudPtr out(new ColorCloud());
  BOOST_FOREACH(ColorPoint& pt, in->points) {
    pcl::PointXYZRGB outpt;
    outpt.x = pt.x;
    outpt.y = pt.y;
    outpt.z = pt.z;
    outpt.r = r;
    outpt.g = g;
    outpt.b = b;
    out->push_back(outpt);
  }
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  return out;
}

ColorCloudPtr colorByID(ColorCloudPtr in, int id) {
  uint8_t r = (id * 53)%256;
  uint8_t g = (id * 139)%256;
  uint8_t b = (id * 223)%256;
  return addColor(in, r, g, b);
}


void TabletopTrackerROS::publish() {
  // turnpick_msgs::TabletopClusters tc;
  // turnpick_msgs::Table table;
  // vector<sensor_msgs::PointCloud2> objects;
  // vector<int> objects_id;
  
  // table.x_min = xmin;
  // table.x_max = xmax;
  // table.y_min = ymin;
  // table.y_max = ymax;

  for (int i=0; i < clusters.size(); i++) {
    sensor_msgs::PointCloud2 pc;
    ColorCloudPtr coloredCloud = colorByID(clusters[i],ids[i]);
    pcl::toROSMsg(*coloredCloud,pc);
    pc.header.frame_id = "base_footprint";
    pc.header.stamp = ros::Time(0);
    cloud_pub.publish(pc);
  }

  // tc.table = table;
  // tc.objects = objects;
  // tc.objects_id = objects_id;

}
