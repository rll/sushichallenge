#include "tracker.h"
#include "table.h"
#include "utils_pcl.h"
#include "cloud_ops.h"
#include <boost/foreach.hpp>
#include "dist_math.h"
#include <ros/console.h>
#include <sstream>
#include "spinning_table_config.h"

using namespace std;
using namespace Eigen;

void TabletopTracker::setLatest(ColorCloudPtr cloud) {
  latest_cloud = cloud;
};

void TabletopTracker::updateCloud() {
  transformed_cloud = transformPointCloud1(downsampleCloud(latest_cloud, SpinConfig::VOXEL_SIZE), transform);
}

void TabletopTracker::updateTable() {
  ROS_DEBUG_STREAM(transform.matrix());
  table_height = getTableHeight(transformed_cloud);
  ROS_INFO_STREAM("table_height " << table_height);
  ColorCloudPtr in_table = getTablePoints(transformed_cloud, table_height);
  in_table = getBiggestCluster(in_table, SpinConfig::TABLE_CLUSTERING_TOLERANCE);
  table_hull = findConvexHull(in_table, table_polygons);
  getTableBounds(in_table, xmin, xmax, ymin, ymax);
  //  fixZ(table_hull, table_height);

}

std::vector<int> arange(int lo, int hi, int step) {
  int n = (hi - lo) / step;
  std::vector<int> out(n);
  for (int i=0; i < n; i++) out[i] = lo + step*i;
  return out;
}


vector<ColorCloudPtr> mergeOverlappingCircles(vector<ColorCloudPtr> clu_list) {
  int n_clu = clu_list.size();
  vector<VectorXf> params_list(n_clu);
  vector<int> merge_to(n_clu);

  merge_to = arange(0, n_clu, 1);

  for (int i=0; i < n_clu; i++) {
    params_list[i] = getEnclosingCircle(clu_list[i]);
  }

  for (int i=0; i < n_clu; i++)
    for (int j=i+1; j < n_clu; j++)
      {
	float dist = (params_list[i].block(0,0,2,1) - params_list[j].block(0,0,2,1)).norm();
	float rad_sum = params_list[i][3] + params_list[j][3];
	if (dist < rad_sum) merge_to[j] = merge_to[i];
      }


  vector<ColorCloudPtr> merges(n_clu);
  for (int i = 0; i < n_clu; i++) {
    if(!merges[merge_to[i]]) merges[merge_to[i]].reset(new ColorCloud());
    *merges[merge_to[i]] += *clu_list[i];
  }


  vector<ColorCloudPtr> final;
  BOOST_FOREACH(ColorCloudPtr cloud, merges) if (cloud) final.push_back(cloud);

  return final;

}

vector<VectorXf> getCircleParams(vector<ColorCloudPtr> clu_list) {
  vector<VectorXf> params_list(clu_list.size());
  for (int i=0; i < clu_list.size(); i++) {
    params_list[i] = getEnclosingCircle(clu_list[i]);
  }
  return params_list;
}


void TabletopTracker::updateClusters() {
  ColorCloudPtr on_table = getPointsOnTableHull(transformed_cloud, table_hull, table_polygons, table_height+SpinConfig::ABOVE_TABLE_CUTOFF);

  //ColorCloudPtr on_table = filterXYZ(transformed_cloud, xmin, xmax, ymin, ymax, table_height+SpinConfig::ABOVE_TABLE_CUTOFF, 1000);
  if (on_table->size() < 30) throw runtime_error("not enough points on table");
  cout << "on table: " << on_table->size() << endl;
  vector< vector<int> > cluster_inds = findClusters(on_table,SpinConfig::OBJECT_CLUSTERING_TOLERANCE,SpinConfig::OBJECT_CLUSTER_MIN_SIZE);
  if (cluster_inds.size() == 0) throw runtime_error("no reasonably big clusters found on table");

  clusters.clear();
  BOOST_FOREACH(vector<int>& inds, cluster_inds) {
    clusters.push_back(extractInds(on_table, inds));
  }

  clusters = mergeOverlappingCircles(clusters);

}

void TabletopTracker::updateCylinders() {

  MatrixXf new_circle_centers(clusters.size(), 2);
  cylinder_params = getCircleParams(clusters);
  for (int i=0; i < clusters.size(); i++) {
    new_circle_centers(i,0) = cylinder_params[i](0);
    new_circle_centers(i,1) = cylinder_params[i](1);
  }

  if (ids.size() == 0) { // first time 
    ids = arange(0, clusters.size(), 1);
    smallest_unused_id = ids.size();
  }
  else {
    MatrixXf dists = pairwiseSquareDist(new_circle_centers, circle_centers).array().sqrt();
    vector<int> new2old = argminAlongRows(dists);
    vector<int> newids(clusters.size());
    for (int i=0; i < clusters.size(); i++) {
      if (dists(i, new2old[i]) < SpinConfig::OBJECT_MATCH_TOLERANCE)  {
	newids[i] = ids[new2old[i]];
      }
      else {
	ROS_INFO("lost a cluster!");
	newids[i] = smallest_unused_id;
	smallest_unused_id++;
      }
    }
    ids = newids;
  }
  circle_centers = new_circle_centers;

  stringstream ss;
  ss << "ids: ";
  BOOST_FOREACH(int i, ids) ss << i << " ";
  ROS_INFO_STREAM(ss);
}


void TabletopTracker::updateAll() {

  updateTransform();
  updateCloud();
  if (!initialized) {
    updateTable();
    updateClusters();
    updateCylinders();
    initialized = true;
  }
  else {
    updateClusters();
    updateCylinders();
  }

}

void TabletopTracker::reset() {
  clusters = vector<ColorCloudPtr>();
  ids = std::vector<int>();
  smallest_unused_id = 0;
  circle_centers = Eigen::MatrixXf();
}
