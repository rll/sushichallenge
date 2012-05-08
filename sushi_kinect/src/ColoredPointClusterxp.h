#include <pcl/impl/point_types.hpp>


using namespace std;
using namespace pcl;



class ColoredPointClusterxp {
public:

	vector<pcl::PointXYZRGB> points;
	PointXYZRGB center;
	PointXYZRGB variances;
	pcl::PointXYZ boxDimensions;
	double boxOrientation;
	vector<pcl::PointXYZ> boundingBoxPoints;


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
		calculateMoments();
		boxDimensions.x = boxDimensions.y = boxDimensions.z = 0.0;
		boxOrientation = 0.0;
		boundingBoxPoints.clear();
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
		center.x = center.y = center.z = center.r = center.g = center.b = 0.0;
		PointXYZRGB help;
		help.x = help.y = help.z = help.r = help.g = help.b = 0.0;


		for (size_t i = 0; i < points.size(); i++) {
			center.x += points.at(i).x;
			center.y += points.at(i).y;
			center.z += points.at(i).z;
			center.r += points.at(i).r;
			center.g += points.at(i).g;
			center.b += points.at(i).b;

		}		
		if (points.size() > 0) {
			center.x /= points.size();
			center.y /= points.size();
			center.z /= points.size();
			center.r /= points.size();
			center.g /= points.size();
			center.b /= points.size();
		}

		for (size_t i = 0; i < points.size(); i++) {	//variance calculation
			help.x = help.x + (points.at(i).x - center.x) * (points.at(i).x - center.x);
			help.y = help.y + (points.at(i).y - center.y) * (points.at(i).y - center.y);
			help.z = help.z + (points.at(i).z - center.z) * (points.at(i).z - center.z);
			help.r = help.r + (points.at(i).r - center.r) * (points.at(i).r - center.r);
			help.g = help.g + (points.at(i).g - center.g) * (points.at(i).g - center.g);
			help.b = help.b + (points.at(i).b - center.b) * (points.at(i).b - center.b);

		}
		if (points.size() > 0) {
			variances.x = help.x / points.size();
			variances.y = help.y / points.size();
			variances.z = help.z / points.size();
			variances.r = help.r / points.size();
			variances.g = help.g / points.size();
			variances.b = help.b / points.size();
		}

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

	
	void calcBoundingBox() {
		boundingBoxPoints.clear();
		double a00 = 0.0; double a01 = 0.0;
		double a10 = 0.0; double a11 = 0.0;
		double p = 0; double q = 0; double lambda_0 = 0; double lambda_1 = 0;
		double v0x = 0.0; double v0y = 0.0;
		double v1x = 0.0; double v1y = 0.0;
		double magn = 0.0;

		PointXYZRGB err; err.x = err.y = 0;

		if (points.size() > 0) {
		for (size_t i = 0; i < points.size(); i++) {
			err.x = points.at(i).x - center.x;
			err.y = points.at(i).y - center.y;
			
			a00 += err.x * err.x; 
			a01 += err.x * err.y ;
			a11 += err.y * err.y; 
		}	
		}
		
		a00 /= points.size(); a10 = a01 = a01 / points.size(); a11 /= points.size();
		
	//	a00 = 2.5; a01 = a10 = 1.5; a11 = 2.5;

		//(a00 - lambda) * (a11 - lambda) - a10 * a01 = 0;
		//a00 * a11 - a11 * lambda - a00 * lambda + lambda^2 -a10*a01 = 0;
		// lambda^2 + (-a11 - a00)*lambda + a00 * a11 - a10^2 = 0;   
		p = (a11 + a00) / 2.0; q = a00 * a11 - a01*a10;
		lambda_0 = p + sqrt(p*p - q);
		lambda_1 = p - sqrt(p*p - q);
		if (lambda_0 < lambda_1) {double help = lambda_1; lambda_1 = lambda_0; lambda_0 = help;}
		
//		a00 * v0x + a01 * v0y = lambda_0 * v0x;
//		a10 * v0x + a11 * v0y = lambda_0 * v0y;		

//		(a00 - lambda_0) * v0x + 	 a01      * v0y  = 0;
//		    a10 	 * v0x + (a11 - lambda_0) * v0y = 0;		

	//	ROS_INFO("Lambda_0 %f Lambda_1 %f | a00 %f, a01 %f, a10 %f, a11 %f ", lambda_0, lambda_1, a00, a01, a10, a11);

		if (lambda_0 != 0) {
		 v0y = a01*a10 / ((a11 - lambda_0) * (a00 - lambda_0));
		 v0x = -(a11 - lambda_0) * v0y / a10; //(a00 - a01 * foo) / lambda_0;
		 magn = magnV2(v0x,v0y);
		 v0x /= magn; v0y /= magn;
		} else {
			v0x = v0y = 0.0;
		}

		if (lambda_1 != 0) {
		 v1y = a01*a10 / ((a11 - lambda_1) * (a00 - lambda_1));
		 v1x = -(a11 - lambda_1) * v1y / a10; //(a00 - a01 * foo) / lambda_1;
		 magn = magnV2(v1x,v1y);
		 v1x /= magn; v1y /= magn;
		} else {
			v1x = v1y = 0.0;
		}

//		ROS_INFO(" --  I Eigen - vector x %f , y %f ", v0x, v0y);
//		ROS_INFO(" -- II Eigen - vector x %f , y %f ", v1x, v1y);

		
		double maxX = 0.0; double maxY = 0.0; double maxZ = 0.0;

		for (size_t i = 0; i < points.size(); i++) {
			if (fabs(projectionV2(v0x, v0y, points.at(i).x - center.x, points.at(i).y - center.y)) > maxX) {
				maxX = fabs(projectionV2(v0x, v0y, points.at(i).x - center.x, points.at(i).y - center.y));
			//	ROS_INFO("1 - VX: %f, VY: %f | vector x %f , y %f , maxX %f ", points.at(i).x - center.x, points.at(i).y - center.y, v0x, v0y, maxX);
			} 
			if (fabs(projectionV2(v1x, v1y, points.at(i).x - center.x, points.at(i).y - center.y)) > maxY) {
				maxY = fabs(projectionV2(v1x, v1y, points.at(i).x - center.x, points.at(i).y - center.y));
			//	ROS_INFO("2 - VX: %f, VY: %f | vector x %f , y %f , maxY %f ", points.at(i).x - center.x, points.at(i).y - center.y, v1x, v1y, maxY);

			} 
			if (fabs(points.at(i).z - center.z) > maxZ) {
				maxZ = fabs(points.at(i).z - center.z);
			}
		}

		//bounding = maxX * (v0x, v0y) + center; and maxY * (v1x, v1y) + center
	
		//ROS_INFO("Bounding Center: %f | %f | %f | Dim: %f | %f | %f | Angle: %f ", center.x, center.y, center.z, maxX, maxY, maxZ, atan2(v0y, v0x));

		// float multi array
		//center.x center.y center.z extend.x, extend.y, extend.z, angle
		
		boxDimensions.x = maxX;
		boxDimensions.y = maxY;
		boxDimensions.z = maxZ;
		boxOrientation = atan2(v0y, v0x);
		boundingBoxPoints.clear();
		pcl::PointXYZ help, toPush;
//1
		help.x = maxX; help.y = maxY; help.z = maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation)  + center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//2
		help.x = maxX; help.y = -maxY; help.z = maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation) + center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//3
		help.x = -maxX; help.y = -maxY; help.z = maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation) + center.x; toPush.y = help.y * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//4
		help.x = -maxX; help.y = maxY; help.z = maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation)+ center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//5
		help.x = maxX; help.y = maxY; help.z = -maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation)+ center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//6
		help.x = maxX; help.y = -maxY; help.z = -maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation)+ center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//7
		help.x = -maxX; help.y = -maxY; help.z = -maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation)+ center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
//8
		help.x = -maxX; help.y = maxY; help.z = -maxZ;
		toPush.x = help.x * cos(boxOrientation) - help.y * sin(boxOrientation)+ center.x; toPush.y = help.x * sin(boxOrientation) + help.y * cos(boxOrientation) + center.y; toPush.z = help.z + center.z;
		boundingBoxPoints.push_back(toPush);
		

		//for (size_t i = 0; i < boundingBoxPoints.size(); i++) {
		//	ROS_INFO(" Dimensions: %f , %f , %f,| %f, %f, %f ", boundingBoxPoints.at(i).x, boundingBoxPoints.at(i).y, boundingBoxPoints.at(i).z, maxX, maxY, maxZ);
		//}

	}


	double projectionV2(double a, double b, double c, double d) {
		return a*c + b*d;
	}


	double magnV2(double x, double y) {
		return sqrt(x*x + y*y);
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


