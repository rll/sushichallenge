#include "tracker.h"
#include "table.h"
#include "utils_pcl.h"
#include "cloud_ops.h"
#include <boost/foreach.hpp>
#include "dist_math.h"

using namespace std;
using namespace Eigen;


static const float VOXEL_SIZE = .01; // for downsampling
static const float TABLE_CLUSTERING_TOLERANCE=.02; // for finding table = biggest cluster at height
static const float OBJECT_CLUSTERING_TOLERANCE=.03; // for clustering objects on table
static const int OBJECT_CLUSTER_MIN_SIZE = 15; // number of points
static const float OBJECT_MATCH_TOLERANCE = .04;
static const float ABOVE_TABLE_CUTOFF=.01;


void TabletopTracker::setLatest(ColorCloudPtr cloud) {
  latest_cloud = cloud;
};

void TabletopTracker::updateCloud() {
  transformed_cloud = transformPointCloud1(downsampleCloud(latest_cloud, VOXEL_SIZE), transform);
}

void TabletopTracker::updateTable() {
  cout << transform.matrix() << endl;
  table_height = getTableHeight(transformed_cloud);
  cout << "table_height " << table_height << endl;
  ColorCloudPtr in_table = getTablePoints(transformed_cloud, table_height);
  in_table = getBiggestCluster(in_table, TABLE_CLUSTERING_TOLERANCE);
  //  table_hull = findConvexHull(in_table, table_polygons);
  getTableBounds(in_table, xmin, xmax, ymin, ymax);
  //  fixZ(table_hull, table_height);

  table_hull.reset(new ColorCloud());
  pcl::PointXYZRGB p;
  p.z = table_height;

  p.x = xmin;
  p.y = ymin;
  table_hull->push_back(p);
  p.x = xmax;
  p.y = ymin;
  table_hull->push_back(p);
  p.x = xmax;
  p.y = ymax;
  table_hull->push_back(p);
  p.x = xmin;
  p.y = ymax;

  table_hull->push_back(p);

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
	float rad_sum = params_list[i][6] + params_list[j][6];
	if (dist < rad_sum) merge_to[j] = merge_to[i];
      }


  vector<ColorCloudPtr> merges(n_clu);
  for (int i = 0; i < n_clu; i++) {
    if(!merges[merge_to[i]]) merges[merge_to[i]].reset(new ColorCloud());
    *merges[merge_to[i]] += *clu_list[i];
  }


  vector<ColorCloudPtr> final;
  BOOST_FOREACH(ColorCloudPtr cloud, merges) if (cloud) final.push_back(cloud);

  // int final_sum = 0;
  // int orig_sum = 0;
  // BOOST_FOREACH(ColorCloudPtr cloud, final) final_sum += cloud->size();
  // BOOST_FOREACH(ColorCloudPtr cloud, clu_list) orig_sum += cloud->size();
  // printf("orig: %i, final:%i\n", orig_sum, final_sum);

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
  //on_table = getPointsOnTableHull(transformed_cloud, table_hull, table_polygons, table_height+ABOVE_TABLE_CUTOFF);

  ColorCloudPtr on_table = filterXYZ(transformed_cloud, xmin, xmax, ymin, ymax, table_height+ABOVE_TABLE_CUTOFF, 1000);
  vector< vector<int> > cluster_inds = findClusters(on_table,OBJECT_CLUSTERING_TOLERANCE,OBJECT_CLUSTER_MIN_SIZE);

  clusters.clear();
  BOOST_FOREACH(vector<int>& inds, cluster_inds) {
    clusters.push_back(extractInds(on_table, inds));
  }

  clusters = mergeOverlappingCircles(clusters);

}

void TabletopTracker::updateCylinders() {
  MatrixXf new_circle_centers(clusters.size(), 2);
  vector<VectorXf> circle_params = getCircleParams(clusters);
  for (int i=0; i < clusters.size(); i++) {
    new_circle_centers.row(i) = circle_params[i].block(0,0,2,1).transpose();
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
      if (dists(i, new2old[i]) < OBJECT_MATCH_TOLERANCE)  {
	newids[i] = ids[new2old[i]];
	cout << " gotit " << endl;
      }
      else {
	cout << "lost a clusters!" << endl;
	newids[i] = smallest_unused_id;
	smallest_unused_id++;
      }
    }
    ids = newids;
  }
  circle_centers = new_circle_centers;

  cout << "ids: ";
  BOOST_FOREACH(int i, ids) cout << i << " ";
  cout << endl;
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
