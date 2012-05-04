#pragma once
#include "utils_pcl.h"
#include <pcl/Vertices.h>

class TabletopTracker {
public:
  bool initialized;

  Eigen::Affine3f transform;

  ColorCloudPtr latest_cloud;
  ColorCloudPtr transformed_cloud;

  ColorCloudPtr table_hull;
  float xmin, xmax, ymin, ymax;
  float table_height;
  std::vector<pcl::Vertices> table_polygons;

  std::vector<ColorCloudPtr> clusters;
  std::vector<int> ids;
  int smallest_unused_id;
  std::vector<Eigen::VectorXf> cylinder_params;
  Eigen::MatrixXf circle_centers;

  TabletopTracker() : initialized(false) {}
  void setLatest(ColorCloudPtr);
  virtual void updateTransform() = 0; // set transform using latest cloud
  void updateCloud(); // update transformed_cloud using transform
  void updateTable(); // find table
  void updateClusters();
  void updateCylinders();
  virtual void updateAll();
  virtual void reset();
};

