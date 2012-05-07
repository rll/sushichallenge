//  <!--node name="find_cluster" pkg="sushi_kinect" type="find_cluster" launch-prefix="gdb -ex run --args"  -->
#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_ros/transforms.h>



/////////////

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include "ColoredPointClusterxp.h"

#include <tf/transform_listener.h>

//#include <geometry_msgs/PoseArray.h>



using namespace std;
namespace enc = sensor_msgs::image_encodings;




class FindCluster
{
	ros::NodeHandle nh_;
	ros::NodeHandle pt_;
	ros::NodeHandle cp;

	ros::NodeHandle private_node_handle_D;
	ros::NodeHandle private_node_handle_X;

	double paramD;
	double paramX;

	double coeffManual[4];


	ros::Subscriber cloud_sub;
	ros::Publisher cloud_pub;

	ros::Subscriber capture_sub;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	ros::NodeHandle n;
	ros::NodeHandle n2;

	ros::Publisher cloud_percept;
	ros::Publisher cloud_vector;

	sensor_msgs::PointCloud2 cloud_voxelized;

	pcl::PointCloud<pcl::PointXYZRGB> cloud_toRosMsg;
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;

	std::vector<pcl::PointXYZRGB> abovePlanePixelSet;
	std::vector<pcl::PointXYZRGB> onPlanePixelSet;

	std::vector<ColoredPointClusterxp> abovePlaneClusterSet;
	std::vector<ColoredPointClusterxp> onPlaneClusterSet;

	tf::TransformListener listener;
	std::string sourceFrameName;

	//ros::NodeHandle pa;
	//ros::Publisher pose_array;

	cv_bridge::CvImagePtr cv_ptr;

	static const double OpeningAngleHorizontal = 34.0 / 180.0 * 3.14159266;
	static const double OpeningAngleVertical = 27.0 / 180.0 * 3.14159266;
	static const int ImageVerticalOffset = -2;
	static const int ImageHorizontalOffset = 0;

	static const double VoxelizeLeafSize = 0.015; //0.02
	static const double maxClusterLength = 0.4;
	static const double minClusterLength = 0.1;
	static const double minDistanceAbovePlane = 0.0125;
	static const double minDistanceUnderPlane = -0.0125;
	static const double doRANSAC = true;


	//Store point cloud data
	sensor_msgs::PointCloud2Ptr cloud_transformed_ptr;



	int cycleCountPcl;
	int cycleCountImg;

	bool trackingIdFlags[64];
	std::vector<ColoredPointClusterxp> lastabovePlaneClusterSet;

	std::vector<ColoredPointClusterxp> lastonPlaneClusterSet;

	bool captureNow;
	bool transformationWorked;
	//bool doRANSAC;

public:


	FindCluster()
	: it_(nh_)
	{
		cloud_transformed_ptr.reset(new sensor_msgs::PointCloud2());
		transformationWorked = false;

		ROS_INFO("SUBSCRIBING");

		capture_sub = cp.subscribe("/bolt/vision/capture", 1, &FindCluster::captureThis, this);

		cloud_sub = pt_.subscribe("cloud_in", 1, &FindCluster::cloudSCb, this);
		image_sub_ = it_.subscribe("image_in", 1, &FindCluster::imageSCb, this);

		image_pub_ = it_.advertise("/bolt/vision/image", 1);


		cloud_percept = n.advertise<sensor_msgs::PointCloud2>("/bolt/vision/biggest_cloud_cluster", 10);
		cloud_vector = n2.advertise<geometry_msgs::Vector3>("/bolt/vision/cloud_vector", 10);



		cycleCountPcl = 0;
		cycleCountImg = 0;

		captureNow = false;
	}

	~FindCluster()
	{
	}





	void reducePointCloudByHeight(sensor_msgs::PointCloud2& cloud2, double height) {


		pcl::PointCloud<pcl::PointXYZRGB> cloud_toRosMsg;
		pcl::fromROSMsg(cloud2, cloud_toRosMsg);


		size_t i = 0;

		while (i < cloud_toRosMsg.points.size()) {

			if (
					(cloud_toRosMsg.points[i].z < height)

			)
			{
				cloud_toRosMsg.erase(cloud_toRosMsg.begin()+i);

			} else {
				i++;
			}
		}
		pcl::toROSMsg(cloud_toRosMsg, cloud2);

	}


	/********************************************************************/

	void captureThis(std_msgs::String msg) {
		captureNow = true;
	}



	/********************************************************************/


	//  Distance from a point x, y, z   to   a plane (a * Xplane + b * YPlane + c * Zplane + d = 0)
	double signedPointPlaneDistance(double x, double y, double z, double a, double b, double c, double d) {
		if ((a == 0) && (b == 0) && (c == 0)) {
			return 0.0;
		}

		double x0, y0, z0;
		x0 = y0 = z0 = 0.0;

		if (d != 0) {
			if (a == 0) {
				if (b == 0) {
					if (c != 0) {
						z0 = -d / c;
					}
				} else {
					y0 = -d / b;
				}
			} else {
				x0 = -d / a;
			}
		}

		double sign = 1.0;
		if ((a*x + b*y + c*z + d) < 0) {sign = -1.0; }

		return sign * (fabs(a*(x - x0) + b*(y - y0) + c*(z - z0)) / pow((a*a) + (b*b) + (c*c), 0.5));
	}

	/********************************************************************/


	double vectorLength(double x, double y, double z) {
		return sqrt(x*x + y*y + z*z);
	}

	/********************************************************************/

	bool isPointWithinBoundaries(double imgMinX, double imgMaxX, double imgMinY, double imgMaxY, double boundaryX, double boundaryY, double x, double y) {
		return ((x >= imgMinX + boundaryX) && (x < imgMaxX - boundaryX) && (y >= imgMinY + boundaryY) && (y < imgMaxY - boundaryY));
	}


	/********************************************************************/

	void calculateImagePositionFrom3dPoint(double x, double y, double z, int colNumber, int rowNumber,
			double opAngH, double opAngV, int& imgx, int& imgy) {

		imgx = (int)((atan2(x, z) / opAngH + 1) * colNumber / 2) + ImageHorizontalOffset;
		imgy = (int)((atan2(y, z) / opAngV + 1) * rowNumber / 2) + ImageVerticalOffset;
	}


	/********************************************************************/
	std::vector<ColoredPointClusterxp> createClusterSet(std::vector<pcl::PointXYZRGB> coloredPointSet) {
		std::vector<ColoredPointClusterxp> abovePlaneClusterSet;
		ColoredPointClusterxp cluster;

		for (size_t i = 0; i < coloredPointSet.size (); i++) {
			cluster.clear();
			cluster.addElement(coloredPointSet.at(i));
			//if (i % 100 == 0) {std::cerr << " createClusterSet, i " << i << " x: " << coloredPointSet.at(i).x << std::endl;}

			abovePlaneClusterSet.push_back(cluster);
		}
		return abovePlaneClusterSet;
	}


	/********************************************************************/
	std::vector<ColoredPointClusterxp> avgLinkageClusterSet(std::vector<ColoredPointClusterxp> abovePlaneClusterSetIn, double distWeight, double colWeight, double distanceBound) {
		for (size_t i = 0; i < abovePlaneClusterSetIn.size (); i++) {
			size_t j = i + 1;
			while (j < abovePlaneClusterSetIn.size ()) {
				if (abovePlaneClusterSetIn.at(i).avgClusterDistanceNorm(abovePlaneClusterSetIn.at(j), distWeight, colWeight) <= distanceBound) {
					abovePlaneClusterSetIn.at(i).mergeWithCluster(abovePlaneClusterSetIn.at(j)); //merge
					abovePlaneClusterSetIn.erase(abovePlaneClusterSetIn.begin()+j);  // ... delete,
				}
				else {
					j++;
				}
			}
		}
		return abovePlaneClusterSetIn;
	}


	/********************************************************************/
	void singleLinkageClusterSet(std::vector<ColoredPointClusterxp>& clusterSetIn, double distWeight, double colWeight, double distanceBound) {
		for (size_t i = 0; i < clusterSetIn.size (); i++) {
			size_t j = i + 1;
			while (j < clusterSetIn.size ()) {
				if (clusterSetIn.at(i).minClusterDistanceNorm(clusterSetIn.at(j), distWeight, colWeight) <= distanceBound) {
					clusterSetIn.at(i).mergeWithCluster(clusterSetIn.at(j)); //merge
					clusterSetIn.erase(clusterSetIn.begin()+j);  // ... delete,
				}
				else {
					j++;
				}
			}
		}
	}


	/********************************************************************/
	void singleLinkageClusterSetTwoSources(std::vector<ColoredPointClusterxp>& clusterSetIn_A, std::vector<ColoredPointClusterxp>& clusterSetIn_B, double distWeight, double colWeight, double distanceBound) {
		for (size_t i = 0; i < clusterSetIn_A.size (); i++) {
			size_t j = 0;
			while (j < clusterSetIn_B.size ()) {
				if (clusterSetIn_A.at(i).minClusterDistanceNorm(clusterSetIn_B.at(j), distWeight, colWeight) <= distanceBound) {
					clusterSetIn_A.at(i).mergeWithCluster(clusterSetIn_B.at(j)); //merge
					clusterSetIn_B.erase(clusterSetIn_B.begin()+j);  // ... delete,
				}
				else {
					j++;
				}
			}
		}
	}



	/********************************************************************/

	void eraseBigPlaneCluster(std::vector<ColoredPointClusterxp>& clusterSet) {
		for (size_t i = 0; i < clusterSet.size();) {

			if (clusterSet.at(i).getMaxClusterLength() > maxClusterLength) {
				//ROS_INFO("Erasing x: %f y: %f z: %f", clusterSet.at(i).center.x, clusterSet.at(i).center.y, clusterSet.at(i).center.z);
				clusterSet.erase(clusterSet.begin() + i);
			} else {i++;}
		}
	}

	/********************************************************************/

	void eraseSmallPlaneCluster(std::vector<ColoredPointClusterxp>& clusterSet) {
		for (size_t i = 0; i < clusterSet.size(); ) {

			//ROS_INFO("Cluster %d length %f" , i , clusterSet.at(i).getMaxClusterLength());

			if (clusterSet.at(i).getMaxClusterLength() < minClusterLength) {
				//ROS_INFO("Erasing x: %f y: %f z: %f", clusterSet.at(i).center.x, clusterSet.at(i).center.y, clusterSet.at(i).center.z);
				clusterSet.erase(clusterSet.begin() + i);
			} else {i++;}
		}
	}



	/********************************************************************/
	void assignTrackingIds(std::vector<ColoredPointClusterxp>& curabovePlaneClusterSet, std::vector<ColoredPointClusterxp>& lastabovePlaneClusterSet) {
		for (int i = 0; i < 64; i++) {
			trackingIdFlags[i] = false;
		}

		double currentNorm = 0.0;
		double minDist = 1000.0;
		size_t minIndex = 0;
		double minDistThres = 0.5;


		for (size_t i = 0; i < curabovePlaneClusterSet.size(); i++) {
			for (size_t j = 0; j < lastabovePlaneClusterSet.size(); j++) {
				currentNorm = curabovePlaneClusterSet.at(i).clusterDistanceNormForTracking(lastabovePlaneClusterSet.at(j), 1.0, 0.0);
				minDist = 1000; minIndex = 0;
				if (currentNorm < minDist) {
					minDist = currentNorm;
					minIndex = j;
				}
			}

			if (lastabovePlaneClusterSet.size() == 0) {
				break;
			}

			if (minDist < minDistThres) {  //found Greedy Match
				if (lastabovePlaneClusterSet.size() > minIndex) {
					curabovePlaneClusterSet.at(i).trackingId = lastabovePlaneClusterSet.at(minIndex).trackingId;
					lastabovePlaneClusterSet.erase(lastabovePlaneClusterSet.begin() + minIndex);
					trackingIdFlags[minIndex%64] = true;
				} else {
					cerr << " ERROR: lastClustersize: " << lastabovePlaneClusterSet.size() << " minIndex " << minIndex <<  std::endl;
				}
			}
		}

		//cerr << " curClustersize: " << curabovePlaneClusterSet.size() << " lastClustersize: " << lastabovePlaneClusterSet.size() <<  std::endl;

		for (size_t i = 0; i < curabovePlaneClusterSet.size(); i++) {
			if (curabovePlaneClusterSet.at(i).trackingId == -1) {
				for (int j = 0; j < 64; j++) {
					if (trackingIdFlags[j] == false) {
						curabovePlaneClusterSet.at(i).trackingId = j;
						trackingIdFlags[j] = true;
						break;
					}
				}
			}
		}
	}


	/********************************************************************/
	void siftExample(cv_bridge::CvImagePtr& cv_ptr)
	{
		//    const cv::Mat input = cv::imread("input.jpg", 0); //Load as grayscale

		//const cv::Mat input = cv::imread("goldengate.jpg", 0); //Load as grayscale

		cv::SiftFeatureDetector detector;
		std::vector<cv::KeyPoint> keypoints;

		//detector.detect(input, keypoints);

		detector.detect(cv_ptr->image, keypoints);


		// Add results to image and save.
		//cv::Mat output;

		std::cerr << "Num of Sift Features: " << keypoints.size() << std::endl;
		//    cv::drawKeypoints(input, keypoints, output);

		cv::drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image);

		//cv::imwrite("sift_result.jpg", output);

	}


	/********************************************************************/
	void dumpOut(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointClusterxp>& abovePlaneClusterSet, std::vector<pcl::PointXYZRGB>& onPlanePixelSet, bool imageOnly) {

		std::stringstream strstr;
		std::stringstream strstr2;
		std::stringstream strstr3;

		time_t seconds;
		seconds = time (NULL);

		if (!imageOnly) {
			strstr << "/home/goehring/ros_workspace/testbolt/ros/vision/kinect/" << seconds/*asctime(gmtime(&seconds))*/ << "_rawImage.bmp";
			cv::imwrite(strstr.str().c_str(), cv_ptr->image);

			ofstream myfile;
			strstr2 << "/home/goehring/ros_workspace/testbolt/ros/vision/kinect/" << seconds << "_points_XYZ_RGB_class.txt";
			myfile.open (strstr2.str().c_str());

			for (size_t j = 0; j < onPlanePixelSet.size(); j++) {
				myfile << onPlanePixelSet.at(j).x << ","<< onPlanePixelSet.at(j).y <<","<< onPlanePixelSet.at(j).z <<","<< onPlanePixelSet.at(j).r << ","<< onPlanePixelSet.at(j).g <<","<< onPlanePixelSet.at(j).b <<","<< 0 <<"\n";
			}

			for (size_t i = 0; i < abovePlaneClusterSet.size(); i++) {

				for (size_t j = 0; j < abovePlaneClusterSet.at(i).points.size(); j++) {
					myfile << abovePlaneClusterSet.at(i).points.at(j).x << ","<<abovePlaneClusterSet.at(i).points.at(j).y <<","<<abovePlaneClusterSet.at(i).points.at(j).z <<","<< abovePlaneClusterSet.at(i).points.at(j).r << ","<<abovePlaneClusterSet.at(i).points.at(j).g <<","<<abovePlaneClusterSet.at(i).points.at(j).b<<","<<i + 1<<"\n";
				}
			}

			myfile.close();
		} else {
			strstr3 << "/home/goehring/ros_workspace/testbolt/ros/vision/kinect/" << seconds << "_debugImage.bmp";
			cv::imwrite(strstr3.str().c_str() , cv_ptr->image);
		}

	}



	/********************************************************************/

	int countPointsInProximity(pcl::PointCloud<pcl::PointXYZRGB>& abovePlanePixelSet, double prox, double c0, double c1, double c2, double c3) {
		int count = 0;
		//ROS_INFO("Starting: %d ", (int)abovePlanePixelSet.size());
		for (size_t i = 0; i < abovePlanePixelSet.size(); i++) {
			//ROS_INFO("X: %f | Y: %f | Z: %f", abovePlanePixelSet.at(i).x, abovePlanePixelSet.at(i).y, abovePlanePixelSet.at(i).z);
			if (fabs(signedPointPlaneDistance(abovePlanePixelSet.at(i).x, abovePlanePixelSet.at(i).y, abovePlanePixelSet.at(i).z, c0, c1, c2, c3)) < prox) {
				count++;
			}
		}

		//ROS_INFO("In Count %f", count);
		return count;
	}




	/********************************************************************/

	void cloudSCb(const sensor_msgs::PointCloud2ConstPtr& input)
	{

		try{
			transformationWorked = pcl_ros::transformPointCloud("/base_link", *input, *cloud_transformed_ptr, listener);
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("Transform Exception in Kinect find cluster");
		}

		if (cycleCountPcl%10 == 0) {
			ROS_INFO(":CloudSCb: New Pcl Data Received ");
		}
		if (transformationWorked) {
			//ROS_INFO(":CloudSCb: Transformation worked ");
			
		} else {
			ROS_INFO(":CloudSCb: Transformation Failed ");
			return;
			//sor.setInputCloud (input); //USED TO BE INPUT
		}

	//clear clusterSets
		abovePlanePixelSet.clear();
		onPlanePixelSet.clear();
		abovePlaneClusterSet.clear();
		onPlaneClusterSet.clear();

		sourceFrameName = std::string(input->header.frame_id);
	
	//Voxelization continues
		sor.setInputCloud (cloud_transformed_ptr); //USED TO BE INPUT
		sor.setLeafSize (VoxelizeLeafSize, VoxelizeLeafSize, VoxelizeLeafSize);
		sor.filter (cloud_voxelized);

		cycleCountPcl++;

	// delete all points closer 0.5 m to the ground
		reducePointCloudByHeight(cloud_voxelized, 0.50);

	//converte to RosPointCloud
		pcl::fromROSMsg (cloud_voxelized, cloud_toRosMsg);


		double stepsize_1 = 0.1; 
		int maxLocalNr_1 = 0; 
		double maxLocalHeight_1 = 0;
		double startHeight_1 = 0.2;
		double stopHeight_1 = 1.8;

		double vA = 0.0; double vB = 0.0; double vC = 1; double vD = 0; 

		for (double height_1 = startHeight_1; height_1 < stopHeight_1; height_1 += stepsize_1) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.1, vA, vB, vC, -height_1);
			if (number > maxLocalNr_1) {maxLocalHeight_1 = height_1; maxLocalNr_1 = number;}
			//ROS_INFO("1 - Points in height %f, proximity %f, number %d ", height_1, stepsize_1, number);
		}
		//ROS_INFO("1 - Height: %f", maxLocalHeight_1);

		double stepsize_2 = 0.01; 
		int maxLocalNr_2 = 0; 
		double maxLocalHeight_2 = 0;

		for (double height_2 = maxLocalHeight_1 - stepsize_1; height_2 < maxLocalHeight_1 + stepsize_1; height_2 += stepsize_2) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.01, vA, vB, vC, -height_2);
			if (number > maxLocalNr_2) {maxLocalHeight_2 = height_2; maxLocalNr_2 = number;}
			//ROS_INFO("2 - Points in height %f, proximity %f, number %d ", height_2, stepsize_2, number);
		}
		//ROS_INFO("2.0 - Height: %f , Points: %d", maxLocalHeight_2, maxLocalNr_2);

		vD = -maxLocalHeight_2;

		// x - Axis -----------------------

		double xAngleStep = 0.01; 
		int xMaxPointsForAngle = 0;
		double xBestLocalAngle = 0;
		double xAngleStartValue = -0.05; 
		double xAngleStopValue = 0.05; 

		for (double xAngle = xAngleStartValue; xAngle < xAngleStopValue; xAngle += xAngleStep) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.01, vA, cos(xAngle)*vB-sin(xAngle)*vC, sin(xAngle)*vB+cos(xAngle)*vC, vD);
			if (number > xMaxPointsForAngle) {xBestLocalAngle = xAngle; xMaxPointsForAngle = number;}
			//		ROS_INFO("xxxxx 3 - Points in angle %f, number %d ", xBestLocalAngle, xMaxPointsForAngle);

		}

		//	ROS_INFO("3 - Points in angle %f, xMaxPointsForAngle %d ", xBestLocalAngle, xMaxPointsForAngle);



		//		0		1	0		0		0
		//		-sin(a)	=	0	cos(a)    -sin(a)	*	0
		//		cos(a)		0	sin(a)	   cos(a)		1

		vB =  cos(xBestLocalAngle)*vB-sin(xBestLocalAngle)*vC; 
		vC =  sin(xBestLocalAngle)*vB+cos(xBestLocalAngle)*vC;


		// - Height Again 1

		stepsize_2 = 0.01;
		maxLocalNr_2 = 0;
		maxLocalHeight_2 = -vD;

		for (double height_2 = maxLocalHeight_2 - stepsize_1; height_2 < maxLocalHeight_2 + stepsize_1; height_2 += stepsize_2) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.01, vA, vB, vC, -height_2);
			if (number > maxLocalNr_2) {maxLocalHeight_2 = height_2; maxLocalNr_2 = number;}
		}
		//	ROS_INFO("2.1 - Height: %f", maxLocalHeight_2);

		vD = -maxLocalHeight_2;




		// y - Axis -----------------------

		double yAngleStep = 0.01; 
		int yMaxPointsForAngle = 0;
		double yBestLocalAngle = 0;
		double yAngleStartValue = -0.05; 
		double yAngleStopValue = 0.05; 

		for (double yAngle = yAngleStartValue; yAngle < yAngleStopValue; yAngle += yAngleStep) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.01, cos(yAngle)*vA-sin(yAngle)*vC, vB, sin(yAngle)*vA+cos(yAngle)*vC, vD);
			if (number > yMaxPointsForAngle) {yBestLocalAngle = yAngle; yMaxPointsForAngle = number;}
		}

		//	ROS_INFO("4 - Points in angle %f, yMaxPointsForAngle %d ", yBestLocalAngle, yMaxPointsForAngle);



		//		-sin(a)		cos(a)	0	  -sin(a)		0
		//		0	=	0	1	       0	*	0
		//		cos(a)		sin(a)	0	   cos(a)		1

		vA =  cos(yBestLocalAngle)*vA-sin(yBestLocalAngle)*vC; 
		vC =  sin(yBestLocalAngle)*vA+cos(yBestLocalAngle)*vC;


		// - Height Again 2

		stepsize_2 = 0.01;
		maxLocalNr_2 = 0;
		maxLocalHeight_2 = -vD;

		for (double height_2 = maxLocalHeight_2 - stepsize_1; height_2 < maxLocalHeight_2 + stepsize_1; height_2 += stepsize_2) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.01, vA, vB, vC, -height_2);
			if (number > maxLocalNr_2) {maxLocalHeight_2 = height_2; maxLocalNr_2 = number;}
		}
		//	ROS_INFO("2.2 - Height: %f", maxLocalHeight_2);

		vD = -maxLocalHeight_2;


		coeffManual[0] = vA;
		coeffManual[1] = vB;
		coeffManual[2] = vC;
		coeffManual[3] = vD;


		/* SAC Starts */
		/*

		pcl::ModelCoefficients coefficients;
		pcl::PointIndices inliers;
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.05); //0.01

		seg.setInputCloud (cloud_toRosMsg.makeShared ());

		seg.segment (inliers, coefficients);

		if (coefficients.values.size() > 3) {
			coeffManual[0] = coefficients.values.at(0);
			coeffManual[1] = coefficients.values.at(1);
			coeffManual[2] = coefficients.values.at(2);
			coeffManual[3] = coefficients.values.at(3);
		} else {
			ROS_INFO("RANSAC FAILED");
		}

		 */
		/* SAC End */



		pcl::PointXYZRGB rgbdPixel;
		pcl::PointXYZRGB planeRgbdPixel;

		//if (cycleCountImg%100 == 0) {
		//	ROS_INFO("A: %f B: %f C: %f D: %f Cyc %d", coeffManual[0], coeffManual[1], coeffManual[2], coeffManual[3], cycleCountImg);
		//}

		for (size_t i = 0; i < cloud_toRosMsg.points.size (); i++)
		{
			//ignore everything, matching the following criterion
			double signedPointDistanceToSACPlane = signedPointPlaneDistance(cloud_toRosMsg.points[i].x, cloud_toRosMsg.points[i].y, cloud_toRosMsg.points[i].z, coeffManual[0], coeffManual[1], coeffManual[2], coeffManual[3]);

			if (cloud_toRosMsg.points[i].z < 1.5) {
				if (signedPointDistanceToSACPlane >= minDistanceAbovePlane) { // if points are on the table and a bit under - fill colorpointarray
					rgbdPixel.x = cloud_toRosMsg.points[i].x;
					rgbdPixel.y = cloud_toRosMsg.points[i].y;
					rgbdPixel.z = cloud_toRosMsg.points[i].z;
					rgbdPixel.r = cloud_toRosMsg.points[i].r;
					rgbdPixel.g = cloud_toRosMsg.points[i].g;
					rgbdPixel.b = cloud_toRosMsg.points[i].b;

					abovePlanePixelSet.push_back(rgbdPixel);

				} else
					if (signedPointDistanceToSACPlane > minDistanceUnderPlane) {
						planeRgbdPixel.x = cloud_toRosMsg.points[i].x;
						planeRgbdPixel.y = cloud_toRosMsg.points[i].y;
						planeRgbdPixel.z = cloud_toRosMsg.points[i].z;
						planeRgbdPixel.r = cloud_toRosMsg.points[i].r;
						planeRgbdPixel.g = cloud_toRosMsg.points[i].g;
						planeRgbdPixel.b = cloud_toRosMsg.points[i].b;

						onPlanePixelSet.push_back(planeRgbdPixel);

					}
			} else {
				continue;	//and if points not above the table
			}

		} // for every Point in cff


		abovePlaneClusterSet = createClusterSet(abovePlanePixelSet);
		onPlaneClusterSet = createClusterSet(onPlanePixelSet);

		ROS_INFO(":CloudSCb: 2 aboveSet: %d, onSet: %d", (int)abovePlaneClusterSet.size(), (int)onPlaneClusterSet.size());

		//double stepSize = 0.02;
		//double upperLimit = 0.2;
		for (int a = 0; ((a < 10) && (abovePlaneClusterSet.size() > 1)); a++) {
			singleLinkageClusterSet(abovePlaneClusterSet, 2.0, 0.001, 0.2);
		}
		eraseBigPlaneCluster(abovePlaneClusterSet);
		eraseSmallPlaneCluster(abovePlaneClusterSet);


		//

		for (int a = 0; ((a < 10) && (onPlaneClusterSet.size() > 1)); a++) {
//			singleLinkageClusterSet(onPlaneClusterSet, 2.0, 0.01, 0.2);

			singleLinkageClusterSet(onPlaneClusterSet, 1.0, 0.001, 0.03);
		}
		eraseBigPlaneCluster(onPlaneClusterSet);
		eraseSmallPlaneCluster(onPlaneClusterSet);


		ROS_INFO(":CloudSCb: 4 aboveSet: %d, onSet: %d", (int)abovePlaneClusterSet.size(), (int)onPlaneClusterSet.size());


		for (int a = 0; ((a < 10) && (abovePlaneClusterSet.size() > 1) && (onPlaneClusterSet.size() > 1)); a ++) {
			singleLinkageClusterSetTwoSources(abovePlaneClusterSet, onPlaneClusterSet, 4.0, 0.0, 0.2);
		}
		eraseBigPlaneCluster(abovePlaneClusterSet);
		eraseSmallPlaneCluster(abovePlaneClusterSet);



		ROS_INFO(":CloudSCb: 6 aboveSet: %d, onSet: %d", (int)abovePlaneClusterSet.size(), (int)onPlaneClusterSet.size());


		//---------------------------------------------------------------A


		//assignTrackingIds(abovePlaneClusterSet, lastabovePlaneClusterSet);
		//assignTrackingIds(onPlaneClusterSet, lastonPlaneClusterSet);


		sensor_msgs::PointCloud2 publishedClusterPC2;
		pcl::PointCloud<pcl::PointXYZRGB> publishCluster;
		pcl::PointXYZRGB helpPoint;

		geometry_msgs::Vector3 bestCluster;
		int maxClusterSize = 0;
		int maxClusterID = 0;


		if (abovePlaneClusterSet.size() > 0) {
			for (size_t i = 0; i < abovePlaneClusterSet.size(); i++) {
				if ((int)abovePlaneClusterSet.at(i).points.size() > maxClusterSize) {
					maxClusterID = i;
					maxClusterSize = abovePlaneClusterSet.at(i).points.size();
				}
			}
			bestCluster.x = abovePlaneClusterSet.at(maxClusterID).center.x;
			bestCluster.y = abovePlaneClusterSet.at(maxClusterID).center.y;
			bestCluster.z = abovePlaneClusterSet.at(maxClusterID).center.z;

			for (size_t j = 0; j < abovePlaneClusterSet.at(maxClusterID).points.size(); j++) {
				helpPoint.x = abovePlaneClusterSet.at(maxClusterID).points.at(j).x;
				helpPoint.y = abovePlaneClusterSet.at(maxClusterID).points.at(j).y;
				helpPoint.z = abovePlaneClusterSet.at(maxClusterID).points.at(j).z;

				helpPoint.r = abovePlaneClusterSet.at(maxClusterID).points.at(j).r;
				helpPoint.g = abovePlaneClusterSet.at(maxClusterID).points.at(j).g;
				helpPoint.b = abovePlaneClusterSet.at(maxClusterID).points.at(j).b;

				publishCluster.push_back(helpPoint);

			}
		} else {
			bestCluster.x = bestCluster.y = 0; bestCluster.z = 1;
		}



		pcl::toROSMsg(publishCluster, publishedClusterPC2);
		publishedClusterPC2.header.frame_id = std::string("/base_link");

		cloud_percept.publish(publishedClusterPC2); // is still empty

		//ROS_INFO(" Best Cluster: X: %f | Y: %f | Z: %f" ,bestCluster.x , bestCluster.y , bestCluster.z);
		//	ROS_INFO("FOO");

		cloud_vector.publish(bestCluster);
		lastabovePlaneClusterSet = abovePlaneClusterSet;

	}




	/********************************************************************/
	void imageSCb(const sensor_msgs::ImageConstPtr& msg)
	{

		if (cycleCountImg%10 == 0)
			std::cerr << "new image data received " << std::endl;

		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//////
		if (cycleCountPcl <= 0) { // no point cloud data
			cycleCountImg++;
			image_pub_.publish(cv_ptr->toImageMsg());
			return;
		}	//else


		//siftExample(cv_ptr);

		if (!transformationWorked) {

			return;
			ROS_INFO("transformation failed");

			//drawClusterPoints(cv_ptr, abovePlaneClusterSet, false);  //only above table
			//drawClusterPoints(cv_ptr, onPlaneClusterSet, true);  //on table
		} else {


		if (captureNow) {
			dumpOut(cv_ptr, abovePlaneClusterSet, onPlanePixelSet, false);
			std::cerr << "Capturing ... Raw" << std::endl;
		}

			drawClusterPoints(cv_ptr, abovePlaneClusterSet, false);  //only above table
			drawClusterPoints(cv_ptr, onPlaneClusterSet, true);  //on table


		}

		if (captureNow) {
			dumpOut(cv_ptr, abovePlaneClusterSet, onPlanePixelSet, true);
			std::cerr << "Capturing ... Debug" << std::endl;
			captureNow = false;
		}

		//---------------------------------------------------------------B

		cycleCountImg++;
		image_pub_.publish(cv_ptr->toImageMsg());



	} //method end

	/********************************************************************/

	void transformClusterSet(std::vector<ColoredPointClusterxp>& sourceClusterSet, std::vector<ColoredPointClusterxp>& targetClusterSet) {

		pcl::PointCloud<pcl::PointXYZRGB> pointCloudSource;
		pcl::PointCloud<pcl::PointXYZRGB> pointCloudTarget;

		ColoredPointClusterxp clusterTarget;
		pcl::PointXYZRGB clusterPoint;

		sensor_msgs::PointCloud2 source, target;

		for (size_t c = 0; c < sourceClusterSet.size(); c++)   {  // for all clusters
			pointCloudSource.clear();
			clusterTarget.clear();		

			for (size_t i = 0; i < sourceClusterSet.at(c).points.size(); i++) {
				clusterPoint = pcl::PointXYZRGB(sourceClusterSet.at(c).points.at(i));
				pointCloudSource.push_back(clusterPoint);
			}

			pcl::toROSMsg(pointCloudSource, source);
			source.header.frame_id = std::string("/base_link");						

			if (pcl_ros::transformPointCloud(sourceFrameName, source, target, listener) == false) {
				if (c == 0) {ROS_INFO("Back Transformation Error");}
			} else {
				//if (c == 0) {ROS_INFO("Back Transformation Success");}
			}	

			pcl::fromROSMsg (target, pointCloudTarget);

			for (size_t i = 0; i < pointCloudTarget.size(); i++) {
				clusterTarget.addElement(pcl::PointXYZRGB(pointCloudTarget.at(i)));
			}

			targetClusterSet.push_back(clusterTarget);
		}

	}



	/********************************************************************/

	void drawClusterPoints(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointClusterxp>& clusterSetOrigin, bool isOnTable = false) {

		std::vector<ColoredPointClusterxp> clusterSet;
		if (transformationWorked) {		
			transformClusterSet(clusterSetOrigin, clusterSet);
		} else {
			clusterSet = clusterSetOrigin;
		}

		int pix_x = 0;
		int pix_y = 0;
		int pix_xf0 = 0;
		int pix_yf0 = 0;
		int pix_xf1 = 0;
		int pix_yf1 = 0;
		int pix_xh0 = 0;
		int pix_yh0 = 0;
		int pix_xh1 = 0;
		int pix_yh1 = 0;

		for (size_t c = 0; c < clusterSet.size(); c++)   {  // for all clusters

			//	if (clusterSet.at(c).points.size() < 10) {   //minClusterSize
			//		continue;
			//	}

			//draw red rectangles around every cluster, then track:
			calculateImagePositionFrom3dPoint(clusterSet.at(c).x0, clusterSet.at(c).y0, clusterSet.at(c).z0, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_xf0, pix_yf0);
			calculateImagePositionFrom3dPoint(clusterSet.at(c).x1, clusterSet.at(c).y1, clusterSet.at(c).z0, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_xf1, pix_yf1);
			calculateImagePositionFrom3dPoint(clusterSet.at(c).x0, clusterSet.at(c).y0, clusterSet.at(c).z1, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_xh0, pix_yh0);
			calculateImagePositionFrom3dPoint(clusterSet.at(c).x1, clusterSet.at(c).y1, clusterSet.at(c).z1, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_xh1, pix_yh1);

			if ((isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, 0, 0, pix_xf0, pix_yf0)) &&
					(isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, 0, 0, pix_xf1, pix_yf1)) &&
					(isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, 0, 0, pix_xh0, pix_yh0)) &&
					(isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, 0, 0, pix_xh1, pix_yh1))) {
				cv::Point pointPx0y0z0(pix_xf0, pix_yf0);
				cv::Point pointPx1y1z0(pix_xf1, pix_yf1);
				cv::Point pointPx0y0z1(pix_xh0, pix_yh0);
				cv::Point pointPx1y1z1(pix_xh1, pix_yh1);

				if (!isOnTable) {
					cv::rectangle(cv_ptr->image, pointPx0y0z0, pointPx1y1z0, CV_RGB(255, 0, 0));
					cv::rectangle(cv_ptr->image, pointPx0y0z1, pointPx1y1z1, CV_RGB(0, 0, 0));
					cv::line(cv_ptr->image, pointPx0y0z0, pointPx0y0z1, CV_RGB(127, 0, 0));
					cv::line(cv_ptr->image, pointPx1y1z0, pointPx1y1z1, CV_RGB(127, 0, 0));
				} else {
					cv::rectangle(cv_ptr->image, pointPx0y0z0, pointPx1y1z0, CV_RGB(0, 0, 255));
					cv::rectangle(cv_ptr->image, pointPx0y0z1, pointPx1y1z1, CV_RGB(0, 0, 0));
					cv::line(cv_ptr->image, pointPx0y0z0, pointPx0y0z1, CV_RGB(0, 0, 127));
					cv::line(cv_ptr->image, pointPx1y1z0, pointPx1y1z1, CV_RGB(0, 0, 127));
				}
			}



			for (size_t i = 0; i < clusterSet.at(c).points.size(); i++) {
				calculateImagePositionFrom3dPoint(clusterSet.at(c).points.at(i).x, clusterSet.at(c).points.at(i).y, clusterSet.at(c).points.at(i).z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x, pix_y);

				int pixs = 1;

				if (isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, pixs, pixs, pix_x, pix_y)) {
					cv::Point pointP1(pix_x-pixs, pix_y-pixs);
					cv::Point pointP2(pix_x+pixs, pix_y+pixs);

					clusterSet.at(c).trackingId = c;

					//					ROS_INFO("C: %d , X %d , Y %d, Id %d ", c, pix_x, pix_y, foo);

					if (clusterSet.at(c).trackingId % 8 == 0) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 0, 255));
					if (clusterSet.at(c).trackingId % 8 == 1) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 255, 0));
					if (clusterSet.at(c).trackingId % 8 == 2) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 255, 0));
					if (clusterSet.at(c).trackingId % 8 == 3) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 0, 255));
					if (clusterSet.at(c).trackingId % 8 == 4) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(128, 255, 0));
					if (clusterSet.at(c).trackingId % 8 == 5) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 255, 255));
					if (clusterSet.at(c).trackingId % 8 == 6) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(128, 0, 255));
					if (clusterSet.at(c).trackingId % 8 == 7) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 128, 255));

				} // if point is in image
			} // for all points


			int pix_x1, pix_x2, pix_y1, pix_y2;

			calculateImagePositionFrom3dPoint(clusterSet.at(c).center.x-sqrt(clusterSet.at(c).variances.x), clusterSet.at(c).center.y-sqrt(clusterSet.at(c).variances.y), clusterSet.at(c).center.z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x1, pix_y1);

			calculateImagePositionFrom3dPoint(clusterSet.at(c).center.x+sqrt(clusterSet.at(c).variances.x), clusterSet.at(c).center.y+sqrt(clusterSet.at(c).variances.y), clusterSet.at(c).center.z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x2, pix_y2);

			int pixs = 0;
			if ((isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, pixs, pixs, pix_x1, pix_y1)) &&
					(isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, pixs, pixs, pix_x2, pix_y2)))
			{
				cv::Point pointP1(pix_x1 - pixs, pix_y1 - pixs);
				cv::Point pointP2(pix_x2 + pixs, pix_y2 + pixs);
				if (clusterSet.at(c).trackingId % 8 == 0) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 0, 255));
				if (clusterSet.at(c).trackingId % 8 == 1) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 255, 0));
				if (clusterSet.at(c).trackingId % 8 == 2) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 255, 0));
				if (clusterSet.at(c).trackingId % 8 == 3) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 0, 255));
				if (clusterSet.at(c).trackingId % 8 == 4) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(128, 255, 0));
				if (clusterSet.at(c).trackingId % 8 == 5) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 255, 255));
				if (clusterSet.at(c).trackingId % 8 == 6) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(128, 0, 255));
				if (clusterSet.at(c).trackingId % 8 == 7) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 128, 255));
			}
		} // for all clusters
	}

}; //class ends



int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter_plane");

	FindCluster tip;

	ros::spin();
	return 0;
}

