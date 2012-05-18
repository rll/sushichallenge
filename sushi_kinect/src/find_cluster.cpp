//  <!--node name="find_cluster" pkg="sushi_kinect" type="find_cluster" launch-prefix="gdb -ex run --args"  -->
#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <dynamic_reconfigure/server.h>
#include <sushi_kinect/ParametersConfig.h>
#include <boost/mem_fn.hpp>
#include <boost/bind.hpp>

/////////////

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "ColoredPointClusterxp.h"
#include "tabletop_object_detector/TabletopSegmentation.h"


#include <tf/transform_listener.h>

//#include <geometry_msgs/PoseArray.h>



using namespace std;
namespace enc = sensor_msgs::image_encodings;




class FindCluster
{

        //********** Nodehandles *************
        ros::NodeHandle nh_;
	ros::NodeHandle pt_;
	ros::NodeHandle cp;
	ros::NodeHandle private_node_handle_D;
	ros::NodeHandle private_node_handle_X;
	ros::NodeHandle nh_service_table;
        ros::NodeHandle n;
        ros::NodeHandle n_box;

        //********** Subscriber/Receiver *************
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        cv_bridge::CvImagePtr cv_ptr;

        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        ros::Subscriber capture_sub;

        image_transport::Publisher image_pub_;
        ros::Publisher cloud_percept;
        ros::Publisher box_data;

        ros::ServiceServer service;


        //********** PointClouds *************
        sensor_msgs::PointCloud2 cloud_voxelized;
        sensor_msgs::PointCloud2Ptr cloud_transformed_ptr;

        pcl::PointCloud<pcl::PointXYZRGB> cloud_toRosMsg;
        pcl::PointCloud<pcl::PointXYZ> cloud_toRosMsgNoColor;
        pcl::PointCloud<pcl::PointXYZRGB> table_cloud;

        sensor_msgs::PointCloud pc1;
        std::vector<pcl::PointXYZRGB> abovePlanePixelSet;
        std::vector<pcl::PointXYZRGB> onPlanePixelSet;

        std::vector<ColoredPointClusterxp> abovePlaneClusterSet;
        std::vector<ColoredPointClusterxp> onPlaneClusterSet;
        std::vector<ColoredPointClusterxp> lastabovePlaneClusterSet;
        std::vector<ColoredPointClusterxp> lastonPlaneClusterSet;

        pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;


        //********** Variables *****************
        dynamic_reconfigure::Server<sushi_kinect::ParametersConfig> server;
        dynamic_reconfigure::Server<sushi_kinect::ParametersConfig>::CallbackType f;

        geometry_msgs::Point32 point32;

	double paramD;
	double paramX;

	double coeffManual[4];
	double vA, vB, vC, vD;

	tf::TransformListener listener;
	std::string sourceFrameName;
	tf::Quaternion q_table;

	static const double OpeningAngleHorizontal = 34.0 / 180.0 * 3.14159266;
	static const double OpeningAngleVertical = 27.0 / 180.0 * 3.14159266;
	static const int ImageVerticalOffset = -2;
	static const int ImageHorizontalOffset = 0;

	static const double VoxelizeLeafSize = 0.01; //0.015
	static const double maxClusterLength = 0.4;
        static const double minClusterLength = 0.01;
	static const double minDistanceAbovePlane = 0.0125;
	static const double minDistanceUnderPlane = -0.0125;
	static const double doRANSAC = true;

        //receiver of new value
        double distAboveTable;
        double colAboveTable;
        double distOnTable;
        double colOnTable;
        double distMergedTable;
        double colMergedTable;
        double voxelLengthNewValue;

        bool recObjectsAboveTable;
        bool recObjectsOnTable;
        bool mrgTableObjects;
        bool trackingIdFlags[64];
        bool captureNow;
        bool transformationWorked;

	int cycleCountPcl;
	int cycleCountImg;

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
                //cloud_vector = n2.advertise<geometry_msgs::Vector3>("/bolt/vision/cloud_vector", 10);
		box_data = n_box.advertise<std_msgs::Float64MultiArray>("bolt/vision/bounding_box_data", 10);

		service = nh_service_table.advertiseService("find_table", &FindCluster::findTable, this);

		cycleCountPcl = 0;
		cycleCountImg = 0;


                //f = boost::bind<void>(boost::mem_fn(&FindCluster::reconfigureVariables), _1, _2);
                f = boost::bind(&FindCluster::reconfigureVariables, this, _1, _2);
		server.setCallback(f);

                 distAboveTable = 0.42;
                 colAboveTable = 500.0;
                 distOnTable = 0.39;
                 colOnTable = 105;
                 distMergedTable = 0.59;
                 colMergedTable = 150.0;
                 voxelLengthNewValue = 0.014;

		captureNow = false;
	}

	~FindCluster()
	{
	}


	bool findTable(tabletop_object_detector::TabletopSegmentation::Request  &req,
		 tabletop_object_detector::TabletopSegmentation::Response &res )
	{
	  //res.detection.table.pose.header.seq;
	  //res.detection.table.pose.header.stamp;
	  res.table.pose.header.frame_id = std::string("/base_link");


// void calcTableParameters(double a, double b, double c, double d, double prox, double& cx, double& cy, double& cz, double& qx, double& qy, double& qz, double& qw, double& minX, double& maxX, double& minY, double& maxY) {

       calcTableParameters(vA, vB, vC, vD, 0.01, res.table.pose.pose.position.x, res.table.pose.pose.position.y, res.table.pose.pose.position.z, res.table.pose.pose.orientation.x, res.table.pose.pose.orientation.y, res.table.pose.pose.orientation.z, res.table.pose.pose.orientation.w, res.table.x_min, res.table.x_max, res.table.y_min, res.table.y_max);





//	  res.detection.clusters[0].header.seq;
//	  res.detection.clusters[0].header.stamp;
	  //res.detection.clusters[0].header.frame_id  = std::string("/base_link");

	  pc1.points.clear();

//////////////// Clusters above Table	
          for (size_t i = 0; i < abovePlaneClusterSet.size(); i++) {
		  pc1.points.clear();	
		  pc1.header.frame_id = std::string("/base_link");

              for (size_t j = 0; j < abovePlaneClusterSet.at(i).points.size(); j++) {

		point32.x = abovePlaneClusterSet.at(i).points.at(j).x;
		point32.y = abovePlaneClusterSet.at(i).points.at(j).y;
		point32.z = abovePlaneClusterSet.at(i).points.at(j).z;
		//pc1.r = abovePlaneClusterSet.at(i).points.at(j).r;
		//pc1.g = abovePlaneClusterSet.at(i).points.at(j).g;
		//pc1.b = abovePlaneClusterSet.at(i).points.at(j).b;

		pc1.points.push_back(point32);


              }
	      res.clusters.push_back(pc1);

           }

//////////////// Clusters on Table
	   for (size_t i = 0; i < onPlaneClusterSet.size(); i++) {
		  pc1.points.clear();	
		  pc1.header.frame_id = std::string("/base_link");

              for (size_t j = 0; j < onPlaneClusterSet.at(i).points.size(); j++) {

		point32.x = onPlaneClusterSet.at(i).points.at(j).x;
		point32.y = onPlaneClusterSet.at(i).points.at(j).y;
		point32.z = onPlaneClusterSet.at(i).points.at(j).z;
		//pc1.r = abovePlaneClusterSet.at(i).points.at(j).r;
		//pc1.g = abovePlaneClusterSet.at(i).points.at(j).g;
		//pc1.b = abovePlaneClusterSet.at(i).points.at(j).b;

		pc1.points.push_back(point32);
              }
	      res.clusters.push_back(pc1);

           }

	  res.result = 4; //SUCCESS
	  return true;
	}



//*******************************************************

void reconfigureVariables(sushi_kinect::ParametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f",
            config.distAbove, config.colAbove, 
            config.distOn, config.colOn,
            config.distMerge, config.colMerge, config.voxelLength);

  distAboveTable = config.distAbove;
  colAboveTable = config.colAbove;
  distOnTable = config.distOn;
  colOnTable = config.colOn;
  distMergedTable = config.distMerge;
  colMergedTable = config.colMerge;
  voxelLengthNewValue = config.voxelLength;
  recObjectsAboveTable = config.bool_RecognizeObjectsAboveTable;
  recObjectsOnTable = config.bool_RecognizeObjectsOnTable;
  mrgTableObjects = config.bool_MergeTableObjects;
}



//*******************************************************

        void reducePointCloudByHeight(sensor_msgs::PointCloud2& cloud2, double height, bool higher = true) {


		pcl::PointCloud<pcl::PointXYZRGB> cloud_toRosMsg;
		pcl::fromROSMsg(cloud2, cloud_toRosMsg);


		size_t i = 0;

		while (i < cloud_toRosMsg.points.size()) {


                        if (((cloud_toRosMsg.points[i].z < height) && (higher == true)) ||
                                ((cloud_toRosMsg.points[i].z > height) && (higher == false)))
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
        void avgLinkageClusterSet(std::vector<ColoredPointClusterxp>& clusterSetIn, double distWeight, double colWeight, double distanceBound) {
                for (size_t i = 0; i < clusterSetIn.size (); i++) {
			size_t j = i + 1;
                        while (j < clusterSetIn.size ()) {
                            if (clusterSetIn.at(i).avgClusterDistanceNorm(clusterSetIn.at(j), distWeight, colWeight) < distanceBound) {
                                //ROS_INFO("merging %d %d %f %f", i, j, clusterSetIn.size(), distanceBound);
                                clusterSetIn.at(i).mergeWithCluster(clusterSetIn.at(j)); //merge
                                clusterSetIn.erase(clusterSetIn.begin()+j);  // ... delete,
                            }
				else {

                               // ROS_INFO("not merging %d %d %f %f", i, j, (clusterSetIn.at(i).avgClusterDistanceNorm(clusterSetIn.at(j), distWeight, colWeight)), distanceBound);

					j++;
				}
			}
		}
	}


        /********************************************************************/
        void avgLinkageClusterSetTwoSources(std::vector<ColoredPointClusterxp>& clusterSetIn_A, std::vector<ColoredPointClusterxp>& clusterSetIn_B, double distWeight, double colWeight, double distanceBound) {
                for (size_t i = 0; i < clusterSetIn_A.size (); i++) {
                        size_t j = 0;
                        while (j < clusterSetIn_B.size ()) {
                                if (clusterSetIn_A.at(i).avgClusterDistanceNorm(clusterSetIn_B.at(j), distWeight, colWeight) <= distanceBound) {
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
	void singleLinkageClusterSet(std::vector<ColoredPointClusterxp>& clusterSetIn, double distWeight, double colWeight, double distanceBound) {
		for (size_t i = 0; i < clusterSetIn.size (); i++) {
			size_t j = i + 1;
			while (j < clusterSetIn.size ()) {
                                if (clusterSetIn.at(i).minClusterDistanceNorm(clusterSetIn.at(j), distWeight, colWeight) < distanceBound) {
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
				myfile << onPlanePixelSet.at(j).x << ","<< onPlanePixelSet.at(j).y <<","<< onPlanePixelSet.at(j).z <<","<< (int)onPlanePixelSet.at(j).r << ","<< (int)onPlanePixelSet.at(j).g <<","<< (int)onPlanePixelSet.at(j).b <<","<< 0 <<"\n";
			}

			for (size_t i = 0; i < abovePlaneClusterSet.size(); i++) {

				for (size_t j = 0; j < abovePlaneClusterSet.at(i).points.size(); j++) {
					myfile << abovePlaneClusterSet.at(i).points.at(j).x << ","<<abovePlaneClusterSet.at(i).points.at(j).y <<","<<abovePlaneClusterSet.at(i).points.at(j).z <<","<< (int)abovePlaneClusterSet.at(i).points.at(j).r << ","<<(int)abovePlaneClusterSet.at(i).points.at(j).g <<","<<(int)abovePlaneClusterSet.at(i).points.at(j).b<<","<<i + 1<<"\n";
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
        void calcTableBoundaries(pcl::PointCloud<pcl::PointXYZRGB>& table_cloud, double& minX, double& minY, double& maxX, double& maxY) {
            int count = 0;
            for (size_t i = 0; i < table_cloud.size(); i++) {
                if (count == 0) {
                        minX = maxX = table_cloud.at(i).x;
                        minY = maxY = table_cloud.at(i).y;
                }
                if (table_cloud.at(i).x < minX) {minX = (float)table_cloud.at(i).x;}
                if (table_cloud.at(i).x > maxX) {maxX = (float)table_cloud.at(i).x;}
                if (table_cloud.at(i).y < minY) {minY = (float)table_cloud.at(i).y;}
                if (table_cloud.at(i).y > maxY) {maxY = (float)table_cloud.at(i).y;}
                count++;
            }
        }

        /*********************************************************************************/
        void erasePointCloudNotOverTable(pcl::PointCloud<pcl::PointXYZRGB>& cloud_toRosMsg, double vA, double vB, double vC, double vD, double tolerance) {
            calculateTablePointSet(cloud_toRosMsg, table_cloud, vA, vB, vC, vD);
            ROS_INFO(" second Size %d", cloud_toRosMsg.size());
            double minX = 0.0; double minY = 0.0; double maxX = 0.0; double maxY = 0.0;
            calcTableBoundaries(table_cloud, minX, minY, maxX, maxY);
           // ROS_INFO("minX %f maxX %f minY %f maxY %f", minX, maxX, minY, maxY);

            for (size_t i = 0; i < cloud_toRosMsg.size();) {
                if (((cloud_toRosMsg.at(i).x - tolerance)< minX) || ((cloud_toRosMsg.at(i).x + tolerance) > maxX) || ((cloud_toRosMsg.at(i).y - tolerance) < minY) || ((cloud_toRosMsg.at(i).y + tolerance) > maxY)) {
                    cloud_toRosMsg.erase(cloud_toRosMsg.begin()+i);
                } else {
                   // ROS_INFO("x %f, y %f, minX %f, maxX %f, minY %f, maxY %f, tolerance %f ", table_cloud.at(i).x, table_cloud.at(i).y, minX, maxX, minY, maxY, tolerance);
                    i++;
                }

            }
            ROS_INFO(" third Size %d", cloud_toRosMsg.size());

        }



	/********************************************************************/
        void calcTableParameters(double a, double b, double c, double d, double prox, double& cx, double& cy, double& cz, double& qx, double& qy, double& qz, double& qw, float& minX, float& maxX, float& minY, float& maxY) {
		
		pcl::PointXYZRGB cog;
		int count = 0;
		cog.x = cog.y = cog.z = cog.r = cog.g = cog.b = 0.0;

		for (size_t i = 0; i < cloud_toRosMsg.size(); i++) {
			if (fabs(signedPointPlaneDistance(cloud_toRosMsg.at(i).x, cloud_toRosMsg.at(i).y, cloud_toRosMsg.at(i).z, vA, vB, vC, vD)) < prox) {
				if (count == 0) {
					minX = maxX = cloud_toRosMsg.at(i).x;
					minY = maxY = cloud_toRosMsg.at(i).y;
				}

				cog.x += cloud_toRosMsg.at(i).x;
				cog.y += cloud_toRosMsg.at(i).y;
				cog.z += cloud_toRosMsg.at(i).z;
				cog.r += cloud_toRosMsg.at(i).r;
				cog.g += cloud_toRosMsg.at(i).g;
				cog.b += cloud_toRosMsg.at(i).b;

                                if (cloud_toRosMsg.at(i).x < minX) {minX = (float)cloud_toRosMsg.at(i).x;}
                                if (cloud_toRosMsg.at(i).x > maxX) {maxX = (float)cloud_toRosMsg.at(i).x;}
                                if (cloud_toRosMsg.at(i).y < minY) {minY = (float)cloud_toRosMsg.at(i).y;}
                                if (cloud_toRosMsg.at(i).y > maxY) {maxY = (float)cloud_toRosMsg.at(i).y;}

				count++;
			}
		}
		cx = cog.x / count; cy = cog.y / count; cz = cog.z / count;

                qx = 0.0;
                qy = 0.0;
                qz = 0.0;
                qw = 1.0;




                //tf::Quaternion q, q2;. q = tf::createQuaternionFromRPY(0, -3.1415 / 2.0, 0)
		//tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw)
	}



        /*******************************************************************/
        void calculateTablePointSet(pcl::PointCloud<pcl::PointXYZRGB>& pCloud, pcl::PointCloud<pcl::PointXYZRGB>& table_cloud, double vA, double vB, double vC, double vD) {
            table_cloud.clear();
            ROS_INFO("Size **pC %d ", pCloud.size());

            for (size_t i = 0; i < pCloud.size(); i++) {
                if (fabs(signedPointPlaneDistance(pCloud.at(i).x, pCloud.at(i).y, pCloud.at(i).z, vA, vB, vC, vD)) <= 0.01) {
                    table_cloud.push_back(pCloud.at(i));
                }
            }
            ROS_INFO("Size **tC %d ", table_cloud.size());
            }

        /********************************************************************/
        void findTableColorGridCluster(pcl::PointCloud<pcl::PointXYZRGB>& pCloud) {

            int grid1[4][4][4];
            int grid2[4][4][4];


            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++) {
                        grid1[i][j][k] = grid2[i][j][k] = 0;
                    }}}

            for (size_t i = 0; i < pCloud.size(); i++) {
                if (fabs(signedPointPlaneDistance(pCloud.at(i).x, pCloud.at(i).y, pCloud.at(i).z, vA, vB, vC, vD)) > 0.01)
                    continue;

                grid1[(((int)(pCloud.at(i).r)) / 4)%4][(((int)(pCloud.at(i).g)) / 4)%4][(((int)(pCloud.at(i).b)) / 4)%4]++;
              //  grid2[(((int)(pCloud.at(i).r + 32)) / 4)%4][(((int)(pCloud.at(i).g + 32)) / 4)%4][(((int)(pCloud.at(i).b + 32)) / 4)%4]++;
            }

            int i1_max, j1_max, k1_max, count1_max; i1_max = j1_max = k1_max = count1_max = 0;
            //int i2_max, j2_max, k2_max, count2_max; i2_max = j2_max = k2_max = count2_max = 0;

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    for (int k = 0; k < 4; k++) {
                        if (grid1[i][j][k] > count1_max) {count1_max = grid1[i][j][k]; i1_max = i; j1_max = j; k1_max = k;}
                   //     if (grid2[i][j][k] > count2_max) {count2_max = grid2[i][j][k]; i2_max = i; j2_max = j; k2_max = k;}
                    }}}

      //      ROS_INFO(" %d %d %d %d", i1_max, j1_max, k1_max, count1_max);
            //ROS_INFO(" %d %d %d %d", i2_max, j2_max, k2_max, count2_max);


        }



	/********************************************************************/

	void cloudSCb(const sensor_msgs::PointCloud2ConstPtr& input)
	{
            double t = (double)cv::getTickCount();

            t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
         //   std::cout << "Times passed at 0 : " << t << std::endl;


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
                sor.setLeafSize (voxelLengthNewValue, voxelLengthNewValue, voxelLengthNewValue);
		sor.filter (cloud_voxelized);

		cycleCountPcl++;

		// delete all points closer 0.5 m to the ground
                reducePointCloudByHeight(cloud_voxelized, 0.50, true);

          //      t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
          //      std::cout << " Times passed after voxelization : " << t << std::endl;


                //convert to RosPointCloud


                try {
                  pcl::fromROSMsg (cloud_voxelized, cloud_toRosMsg);
                } catch (int e) {
                    pcl::fromROSMsg (cloud_voxelized, cloud_toRosMsgNoColor);
                            for (size_t i = 0; i < cloud_toRosMsgNoColor.size(); i++) {
                                cloud_toRosMsg.at(i).x = cloud_toRosMsgNoColor.at(i).x;
                                cloud_toRosMsg.at(i).y = cloud_toRosMsgNoColor.at(i).y;
                                cloud_toRosMsg.at(i).z = cloud_toRosMsgNoColor.at(i).z;
                                cloud_toRosMsg.at(i).r = cloud_toRosMsg.at(i).g = cloud_toRosMsg.at(i).b = 255;
                            }
                }


		double stepsize_1 = 0.1; 
		int maxLocalNr_1 = 0; 
		double maxLocalHeight_1 = 0;
		double startHeight_1 = 0.2;
		double stopHeight_1 = 1.8;

		vA = 0.0; 
		vB = 0.0; 
		vC = 1.0; 
		vD = 0.0; 

		for (double height_1 = startHeight_1; height_1 < stopHeight_1; height_1 += stepsize_1) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.1, vA, vB, vC, -height_1);
			if (number > maxLocalNr_1) {maxLocalHeight_1 = height_1; maxLocalNr_1 = number;}
			//ROS_INFO("1 - Points in height %f, proximity %f, number %d ", height_1, stepsize_1, number);
		}

		double stepsize_2 = 0.01; 
		int maxLocalNr_2 = 0; 
		double maxLocalHeight_2 = 0;

		for (double height_2 = maxLocalHeight_1 - stepsize_1; height_2 < maxLocalHeight_1 + stepsize_1; height_2 += stepsize_2) {
			int number = countPointsInProximity(cloud_toRosMsg, 0.01, vA, vB, vC, -height_2);
			if (number > maxLocalNr_2) {maxLocalHeight_2 = height_2; maxLocalNr_2 = number;}
			//ROS_INFO("2 - Points in height %f, proximity %f, number %d ", height_2, stepsize_2, number);
		}

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


		q_table = tf::createQuaternionFromRPY(xBestLocalAngle, yBestLocalAngle, 0.0);

		coeffManual[0] = vA;
		coeffManual[1] = vB;
		coeffManual[2] = vC;
		coeffManual[3] = vD;


                //erasePointCloudNotOverTable(cloud_toRosMsg, vA, vB, vC, vD, 0.025); //remove everthing in 25 mm prox.


		pcl::PointXYZRGB rgbdPixel;
		pcl::PointXYZRGB planeRgbdPixel;


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

                //Clusters Above Plane
                if (recObjectsAboveTable) {
                for (int a = 0; ((a < 5) && (abovePlaneClusterSet.size() > 1)); a++) {
                    singleLinkageClusterSet(abovePlaneClusterSet, 1.0 / distAboveTable, 1.0 / colAboveTable, 1.0 / 5.0);
		}
                eraseBigPlaneCluster(abovePlaneClusterSet);
                eraseSmallPlaneCluster(abovePlaneClusterSet);
                }

                //second ontable step
                for (int a = 0; ((a < 5) && (onPlaneClusterSet.size() > 1)); a++) {
                    avgLinkageClusterSet(abovePlaneClusterSet, 1.0 / distMergedTable, 1.0 / colMergedTable, 1.0 / 5.0);
                }


                //////////////Clusters on Plane
                if (recObjectsOnTable) {

                for (int a = 0; ((a < 5) && (onPlaneClusterSet.size() > 1)); a++) {
                    singleLinkageClusterSet(onPlaneClusterSet, 1.0 / distOnTable, 1.0 / colOnTable, 1.0 / 5.0);
		}
                eraseBigPlaneCluster(onPlaneClusterSet);
                eraseSmallPlaneCluster(onPlaneClusterSet);
                }

                //second ontable step
                for (int a = 0; ((a < 5) && (onPlaneClusterSet.size() > 1)); a++) {
                    avgLinkageClusterSet(onPlaneClusterSet, 1.0 / distMergedTable, 1.0 / colMergedTable, 1.0 / 5.0);
                }



                //Merge Table Objects
                if (mrgTableObjects) {
                for (int a = 0; ((a < 5) && (abovePlaneClusterSet.size() > 1) && (onPlaneClusterSet.size() > 1)); a++) {
                    avgLinkageClusterSetTwoSources(abovePlaneClusterSet, onPlaneClusterSet, 1.0 / distMergedTable, 1.0 / colMergedTable, 1.0 / 5.0);
		}
                eraseBigPlaneCluster(abovePlaneClusterSet);
                eraseSmallPlaneCluster(abovePlaneClusterSet);
                }


		sensor_msgs::PointCloud2 publishedClusterPC2;
		pcl::PointCloud<pcl::PointXYZRGB> publishCluster;

		std_msgs::Float64MultiArray bounding_box_data_to_publish;

		pcl::PointXYZRGB helpPoint;

                //geometry_msgs::Vector3 bestCluster;
		int maxClusterSize = 0;
		int maxClusterID = 0;


		if (abovePlaneClusterSet.size() > 0) { 
			for (size_t i = 0; i < abovePlaneClusterSet.size(); i++) {
				abovePlaneClusterSet.at(i).calcBoundingBox();	

				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).center.x);
				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).center.y);
				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).center.z);
				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).boxDimensions.x);
				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).boxDimensions.y);
				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).boxDimensions.z);
				bounding_box_data_to_publish.data.push_back(abovePlaneClusterSet.at(i).boxOrientationAngle);

				//ROS_INFO("B A R 1 %d ", abovePlaneClusterSet.at(i).boundingBoxPoints.size());	
				if ((int)abovePlaneClusterSet.at(i).points.size() > maxClusterSize) {
					maxClusterID = i;
					maxClusterSize = abovePlaneClusterSet.at(i).points.size();
				}
			}


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

		}

		//****************************************
		if (onPlaneClusterSet.size() > 0) {
			for (size_t i = 0; i < onPlaneClusterSet.size(); i++) {
				onPlaneClusterSet.at(i).calcBoundingBox();

				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).center.x);
				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).center.y);
				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).center.z);
				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).boxDimensions.x);
				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).boxDimensions.y);
				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).boxDimensions.z);
				bounding_box_data_to_publish.data.push_back(onPlaneClusterSet.at(i).boxOrientationAngle);
				//ROS_INFO("B A R 2 %d ", onPlaneClusterSet.at(i).boundingBoxPoints.size());		
			}
		}


                pcl::toROSMsg(publishCluster, publishedClusterPC2);
		publishedClusterPC2.header.frame_id = std::string("/base_link");

		cloud_percept.publish(publishedClusterPC2); // is still empty


		box_data.publish(bounding_box_data_to_publish);

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
                         if (recObjectsAboveTable) {
                        drawClusterPoints(cv_ptr, abovePlaneClusterSet, false);  //only above table
                         }

                         if (recObjectsOnTable) {
                        drawClusterPoints(cv_ptr, onPlaneClusterSet, true);  //on table
                        }
			//			for (size_t i = 0; i < abovePlaneClusterSet.size(); i++) {
			//				ROS_INFO("ClusterSize %d , %d " , i, abovePlaneClusterSet.at(i).boundingBoxPoints.size());
			//			}

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
		pcl::PointCloud<pcl::PointXYZ> bbPointCloudSource;
		pcl::PointCloud<pcl::PointXYZRGB> pointCloudTarget;
		pcl::PointCloud<pcl::PointXYZ> bbPointCloudTarget;


		ColoredPointClusterxp clusterTarget;

		pcl::PointXYZRGB clusterPoint;
		pcl::PointXYZ bbPoint;

		sensor_msgs::PointCloud2 source, target;
		sensor_msgs::PointCloud2 bbSource, bbTarget;


		for (size_t c = 0; c < sourceClusterSet.size(); c++)   {  // for all clusters
			pointCloudSource.clear();
			bbPointCloudSource.clear();
			clusterTarget.clear();


			for (size_t i = 0; i < sourceClusterSet.at(c).points.size(); i++) {
				clusterPoint = pcl::PointXYZRGB(sourceClusterSet.at(c).points.at(i));
				pointCloudSource.push_back(clusterPoint);
			}

			//boundingBox
			for (size_t i = 0; i < sourceClusterSet.at(c).boundingBoxPoints.size(); i++) {
				bbPoint = pcl::PointXYZ(sourceClusterSet.at(c).boundingBoxPoints.at(i));
				bbPointCloudSource.push_back(bbPoint);
			}

			pcl::toROSMsg(pointCloudSource, source);
			pcl::toROSMsg(bbPointCloudSource, bbSource);

			source.header.frame_id = std::string("/base_link");						
			bbSource.header.frame_id = std::string("/base_link");						


			if (pcl_ros::transformPointCloud(sourceFrameName, source, target, listener) == false) {
				if (c == 0) {ROS_INFO("Back Transformation Error");}
			} else {
				//if (c == 0) {ROS_INFO("Back Transformation Success");}
			}	

			if (pcl_ros::transformPointCloud(sourceFrameName, bbSource, bbTarget, listener) == false) {
				if (c == 0) {ROS_INFO("Back Transformation Error");}
			} else {
				//if (c == 0) {ROS_INFO("Back Transformation Success");}
			}	


			pcl::fromROSMsg (target, pointCloudTarget);
			pcl::fromROSMsg (bbTarget, bbPointCloudTarget);


			for (size_t i = 0; i < pointCloudTarget.size(); i++) {
				clusterTarget.addElement(pcl::PointXYZRGB(pointCloudTarget.at(i)));
			}


			for (size_t i = 0; i < bbPointCloudTarget.size(); i++) {
				clusterTarget.boundingBoxPoints.push_back(pcl::PointXYZ(bbPointCloudTarget.at(i)));
			}


			//			clusterTarget.boundingBoxPoints = std::vector<pcl::PointXYZ>(bbPointCloudTarget);

			targetClusterSet.push_back(clusterTarget);
		}

	}



	/********************************************************************/

	void drawClusterPoints(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointClusterxp>& clusterSetOrigin, bool isOnTable = false) {

		int colorOffset = 0;
		if (isOnTable) {
			colorOffset = 4;
		}

		std::vector<ColoredPointClusterxp> clusterSet;
		if (transformationWorked) {		
			transformClusterSet(clusterSetOrigin, clusterSet);
		} else {
			clusterSet = clusterSetOrigin;
		}

		int pix_x = 0;
		int pix_y = 0;
		vector<cv::Point> boundingBoxPointsImgPlane;


		for (size_t c = 0; c < clusterSet.size(); c++)   {  // for all clusters
			boundingBoxPointsImgPlane.clear();

			//ROS_INFO(" Numof R3 BP: %d ", clusterSet.at(c).boundingBoxPoints.size());		

			for (size_t p = 0; p < clusterSet.at(c).boundingBoxPoints.size(); p++) {
				calculateImagePositionFrom3dPoint(clusterSet.at(c).boundingBoxPoints.at(p).x, clusterSet.at(c).boundingBoxPoints.at(p).y, clusterSet.at(c).boundingBoxPoints.at(p).z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x, pix_y);			

				//ROS_INFO("c %d Transform p. bbp %d , x %d, y %d " , c, p, pix_x, pix_y);

				if (isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, 0, 0, pix_x, pix_y)) {
					cv::Point pointP(pix_x, pix_y);
					boundingBoxPointsImgPlane.push_back(pointP);
				}

			}


			//ROS_INFO("Numof BBP: %d ", boundingBoxPointsImgPlane.size());		

                        //Draw Bounding boxes

                        int bC_R, bC_G, bC_B;


			if (boundingBoxPointsImgPlane.size() == 8) {
                            if (isOnTable) {
                                bC_R = 0; bC_G = 255; bC_B = 0;
                            }  else {
                                bC_R = 255; bC_G = 0; bC_B = 0;
                            }

                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(0), boundingBoxPointsImgPlane.at(1), CV_RGB(bC_R, bC_G, bC_B));
                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(1), boundingBoxPointsImgPlane.at(2), CV_RGB(bC_R, bC_G, bC_B));
                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(2), boundingBoxPointsImgPlane.at(3), CV_RGB(bC_R, bC_G, bC_B));
                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(3), boundingBoxPointsImgPlane.at(0), CV_RGB(bC_R, bC_G, bC_B));

                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(0), boundingBoxPointsImgPlane.at(4), CV_RGB(bC_R, bC_G, bC_B));
                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(1), boundingBoxPointsImgPlane.at(5), CV_RGB(bC_R, bC_G, bC_B));
                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(2), boundingBoxPointsImgPlane.at(6), CV_RGB(bC_R, bC_G, bC_B));
                                cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(3), boundingBoxPointsImgPlane.at(7), CV_RGB(bC_R, bC_G, bC_B));

				cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(4), boundingBoxPointsImgPlane.at(5), CV_RGB(50, 50, 50));
				cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(5), boundingBoxPointsImgPlane.at(6), CV_RGB(50, 50, 50));
				cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(6), boundingBoxPointsImgPlane.at(7), CV_RGB(50, 50, 50));
				cv::line(cv_ptr->image, boundingBoxPointsImgPlane.at(7), boundingBoxPointsImgPlane.at(4), CV_RGB(50, 50, 50));
			}



			for (size_t i = 0; i < clusterSet.at(c).points.size(); i++) {
				calculateImagePositionFrom3dPoint(clusterSet.at(c).points.at(i).x, clusterSet.at(c).points.at(i).y, clusterSet.at(c).points.at(i).z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x, pix_y);

                                int pixs = 1;

				if (isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, pixs, pixs, pix_x, pix_y)) {
                                        cv::Point pointP1(pix_x, pix_y);
					cv::Point pointP2(pix_x+pixs, pix_y+pixs);

					clusterSet.at(c).trackingId = c;

					//					ROS_INFO("C: %d , X %d , Y %d, Id %d ", c, pix_x, pix_y, foo);

					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 0) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 0, 0));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 1) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(255, 255, 0));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 2) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 255, 0));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 3) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 255, 255));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 4) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 0, 255));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 5) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(128, 0, 0));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 6) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 0, 128));
					if ((clusterSet.at(c).trackingId + colorOffset) % 8 == 7) cv::rectangle(cv_ptr->image, pointP1, pointP2, CV_RGB(0, 128, 0));

				} // if point is in image
			} // for all points


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

