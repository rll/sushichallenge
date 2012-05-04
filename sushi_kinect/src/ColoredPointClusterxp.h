#include <pcl/impl/point_types.hpp>


using namespace std;
using namespace pcl;



class ColoredPointClusterxp {
	public:

	vector<pcl::PointXYZRGB> points;
	PointXYZRGB center;
	PointXYZRGB variances;


	double x0, x1, y0, y1, z0, z1;
	int trackingId;


	ColoredPointClusterxp() {
		clear();
	}
	~ColoredPointClusterxp() {}

	void clear() {
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
		
		for (size_t i = 0; i < points.size(); i++) {
		   center.x += points.at(i).x;
		   center.y += points.at(i).y;
		   center.z += points.at(i).z;
		   center.r += points.at(i).r;
		   center.g += points.at(i).g;
		   center.b += points.at(i).b;

		}		
		   center.x /= points.size(); 
		   center.y /= points.size(); 
		   center.z /= points.size(); 
		   center.r /= points.size(); 
		   center.g /= points.size(); 
		   center.b /= points.size(); 

		
		PointXYZRGB help; help.x = help.y = help.z = help.r = help.g = help.b = 0.0;
		for (size_t i = 0; i < points.size(); i++) {	//variance calculation
			help.x = help.x + (points.at(i).x - center.x) * (points.at(i).x - center.x);
			help.y = help.y + (points.at(i).y - center.y) * (points.at(i).y - center.y);
			help.z = help.z + (points.at(i).z - center.z) * (points.at(i).z - center.z);
			help.r = help.r + (points.at(i).r - center.r) * (points.at(i).r - center.r);
			help.g = help.g + (points.at(i).g - center.g) * (points.at(i).g - center.g);
			help.b = help.b + (points.at(i).b - center.b) * (points.at(i).b - center.b);

		}
		  variances.x = help.x / points.size(); 
		  variances.y = help.y / points.size(); 
		  variances.z = help.z / points.size(); 
		  variances.r = help.r / points.size(); 
		  variances.g = help.g / points.size(); 
		  variances.b = help.b / points.size(); 

		x0 = x1 = center.x; y0 = y1 = center.y; z0 = z1 = center.z;  //reset values to center
		for (size_t i = 0; i < points.size(); i++) {
			if (points.at(i).x < x0) {x0 = points.at(i).x;}
			if (points.at(i).x > x1) {x1 = points.at(i).x;}
			if (points.at(i).y < y0) {y0 = points.at(i).y;}
			if (points.at(i).y > y1) {y1 = points.at(i).y;}
			if (points.at(i).z < z0) {z0 = points.at(i).z;}
			if (points.at(i).z > z1) {z1 = points.at(i).z;}
		}		
			

	}

	void addElement(pcl::PointXYZRGB newElement) {
		points.push_back(newElement);
		calculateMoments();
	}

	void mergeWithCluster(ColoredPointClusterxp& other) {
		this->center.x = (this->center.x * this->points.size() + other.center.x * other.points.size()) / (this->points.size() + other.points.size());
		this->center.y = (this->center.y * this->points.size() + other.center.y * other.points.size()) / (this->points.size() + other.points.size());
		this->center.z = (this->center.z * this->points.size() + other.center.z * other.points.size()) / (this->points.size() + other.points.size());
		this->center.r = (this->center.r * this->points.size() + other.center.r * other.points.size()) / (this->points.size() + other.points.size());
		this->center.g = (this->center.g * this->points.size() + other.center.g * other.points.size()) / (this->points.size() + other.points.size());
		this->center.b = (this->center.b * this->points.size() + other.center.b * other.points.size()) / (this->points.size() + other.points.size());

		while (!other.points.empty()) {
			this->points.push_back(other.points.at(other.points.size()-1));
			other.points.pop_back();
		}
		calculateMoments();
	} 

	double avgClusterDistanceNorm(ColoredPointClusterxp other, double spatialDistanceWeight, double coloredDistanceWeight) {
		//average linkage
		return max(
			sqrt(pow(this->center.x - other.center.x, 2) +  pow(this->center.y - other.center.y, 2) + pow(this->center.z - other.center.z, 2)) * spatialDistanceWeight, 
			sqrt(pow(this->center.r - other.center.r, 2) + pow(this->center.g - other.center.g, 2) + pow(this->center.b - other.center.b, 2))  * coloredDistanceWeight
		); 	
	  }



	double minClusterDistanceNorm(ColoredPointClusterxp other, double spatialDistanceWeight, double coloredDistanceWeight) { 
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


	double clusterDistanceNormForTracking(ColoredPointClusterxp other, double spatialDistanceWeight, double coloredDistanceWeight) {
		//average linkage
		return max(
			sqrt(pow(this->center.x - other.center.x, 2) +  pow(this->center.y - other.center.y, 2) + pow(this->center.z - other.center.z, 2)) * spatialDistanceWeight, 
			sqrt(pow(this->center.r - other.center.r, 2) + pow(this->center.g - other.center.g, 2) + pow(this->center.b - other.center.b, 2))  * coloredDistanceWeight
		); 	
  }
}; // end coloredPointCluster


