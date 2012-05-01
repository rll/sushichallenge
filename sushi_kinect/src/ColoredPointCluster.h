#include "ColoredPoint.h"
#include <pcl/impl/point_types.hpp>


using namespace std;



class ColoredPointCluster {
	public:

	std::vector<ColoredPoint> points;

	std::vector<pcl::PointXYZRGB> pointsXP;

	//pcl::PointXYZRGB pointTest;

	ColoredPoint center;
	pcl::PointXYZRGB centerX;

//	ColoredPoint minBoundary;
//	ColoredPoint maxBoundary;
	ColoredPoint variances;
	pcl::PointXYZRGB variancesX;


	double x0, x1, y0, y1, z0, z1;

	int trackingId;

	int numOfElements;


	ColoredPointCluster() {
		clear();
	}
	~ColoredPointCluster() {}

	void clear() {
		numOfElements = 0;  //DEPRECATED
		center.clear();
		variances.clear(); 
		points.clear();
		trackingId = -1;
		x0 = x1 = y0 = y1 = z0 = z1 = 0; //Boundaries
	}

	double getMaxClusterLength() {
		 if (fabs(x0 - x1) > fabs(y0 - y1)) {
			if (fabs(x0 - x1) > fabs(z0 - z1)) {
				return fabs(x0 - x1);
			} else {
				return fabs(z0 - z1);
			}
		 } else if  (fabs(y0 - y1) > fabs(z0 - z1)) {
				return fabs(y0 - y1);
			} else {
				return fabs(z0 - z1);
			}
	}

	void calculateMoments() {
		center.clear(); 
		numOfElements = points.size();
		for (int i = 0; i < numOfElements; i++) {
		   center = center + points.at(i);
		}		
		center = center * 1.0 / numOfElements;
		
		ColoredPoint help;
		for (int i = 0; i < numOfElements; i++) {	//variance calculation
			help = help + (points.at(i) - center).componentwiseProduct(points.at(i) - center);
		}
		variances = help / numOfElements;
		//calculate min parallel box boundaries
		x0 = x1 = center.x; y0 = y1 = center.y; z0 = z1 = center.z;  //reset values to center
		for (int i = 0; i < numOfElements; i++) {
			if (points.at(i).x < x0) {x0 = points.at(i).x;}
			if (points.at(i).x > x1) {x1 = points.at(i).x;}
			if (points.at(i).y < y0) {y0 = points.at(i).y;}
			if (points.at(i).y > y1) {y1 = points.at(i).y;}
			if (points.at(i).z < z0) {z0 = points.at(i).z;}
			if (points.at(i).z > z1) {z1 = points.at(i).z;}
		}		
			

	}

	void addElement(ColoredPoint newElement) {
		points.push_back(newElement);
		calculateMoments();
	}

	void mergeWithCluster(ColoredPointCluster& other) {
		this->center = (this->center * this->numOfElements + other.center * other.numOfElements) / (this->numOfElements + other.numOfElements);
		//this->minBoundary = this->minBoundary.minim(other.minBoundary); //DEPRECATED
		//this->maxBoundary = this->maxBoundary.maxim(other.maxBoundary); //DEPRECATED

		while (!other.points.empty()) {
			this->points.push_back(other.points.at(other.points.size()-1));
			other.points.pop_back();
		}
		calculateMoments();
	} 

	double avgClusterDistanceNorm(ColoredPointCluster other, double spatialDistanceWeight, double coloredDistanceWeight) {
		//average linkage
		return max(
			sqrt(pow(this->center.x - other.center.x, 2) +  pow(this->center.y - other.center.y, 2) + pow(this->center.z - other.center.z, 2)) * spatialDistanceWeight, 
			sqrt(pow(this->center.r - other.center.r, 2) + pow(this->center.g - other.center.g, 2) + pow(this->center.b - other.center.b, 2))  * coloredDistanceWeight
		); 	
	  }



	double minClusterDistanceNorm(ColoredPointCluster other, double spatialDistanceWeight, double coloredDistanceWeight) { 
		//average linkage
		double minDistance = 1000000;
		double dist = 0;

		for (size_t i = 0; i < this->points.size(); i++) {
			for (size_t j = 0; j < other.points.size(); j++) { 
				dist = 
			max(sqrt(pow(this->points.at(i).x - other.points.at(j).x, 2) +  pow(this->points.at(i).y - other.points.at(j).y, 2) + pow(this->points.at(i).z - other.points.at(j).z, 2)) * spatialDistanceWeight, 
			sqrt(pow(this->points.at(i).r - other.points.at(j).r, 2) + pow(this->points.at(i).g - other.points.at(j).g, 2) + pow(this->points.at(i).b - other.points.at(j).b, 2))  * coloredDistanceWeight);
			if (dist < minDistance) {
				minDistance = dist;
			}
			}
		}
		return minDistance; 
	}


	double clusterDistanceNormForTracking(ColoredPointCluster other, double spatialDistanceWeight, double coloredDistanceWeight) {
		//average linkage
		return max(
			sqrt(pow(this->center.x - other.center.x, 2) +  pow(this->center.y - other.center.y, 2) + pow(this->center.z - other.center.z, 2)) * spatialDistanceWeight, 
			sqrt(pow(this->center.r - other.center.r, 2) + pow(this->center.g - other.center.g, 2) + pow(this->center.b - other.center.b, 2))  * coloredDistanceWeight
		); 	
  }





}; // end coloredPointCluster


