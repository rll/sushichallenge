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

  tf::TransformListener listener;


  //ros::NodeHandle pa;
  //ros::Publisher pose_array; 

  cv_bridge::CvImagePtr cv_ptr;

   static const double OpeningAngleHorizontal = 34.0 / 180.0 * 3.14159266;
   static const double OpeningAngleVertical = 27.0 / 180.0 * 3.14159266;
   static const int ImageVerticalOffset = 20;
   static const int ImageHorizontalOffset = -5;

   static const double VoxelizeLeafSize = 0.03; //0.02
   static const double maxClusterLength = 0.4;
   static const double minDistanceAbovePlane = 0.0125;
   static const double minDistanceUnderPlane = -0.0125;
   static const double doRANSAC = true; 


  //Store point cloud data
  sensor_msgs::PointCloud2Ptr cloud_transformed_ptr;

  sensor_msgs::PointCloud2 cloud_filtered;
  pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
  
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;

  int cycleCountPcl;
  int cycleCountImg;

  bool trackingIdFlags[64];
  std::vector<ColoredPointClusterxp> lastClusterSet;

  std::vector<ColoredPointClusterxp> lastPlaneClusterSet;

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





  void reducePointCloudByDistanceAndHeight(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double distanceBoundary, double heightBoundary) {

				
		size_t i = 0;

		while (i < cloud.points.size()) {

			if (
	 			   (cloud.points[i].z > distanceBoundary) || 
				   (cloud.points[i].y < -heightBoundary)
			   )
				 {

				cloud.erase(cloud.begin()+i);
				
			} else {
				i++;
			}
		}

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
	std::vector<ColoredPointClusterxp> clusterSet;
	ColoredPointClusterxp cluster;

	for (size_t i = 0; i < coloredPointSet.size (); i++) {
		cluster.clear();
		cluster.addElement(coloredPointSet.at(i)); 
		//if (i % 100 == 0) {std::cerr << " createClusterSet, i " << i << " x: " << coloredPointSet.at(i).x << std::endl;}

		clusterSet.push_back(cluster);
	}
	return clusterSet;
  }


/********************************************************************/
std::vector<ColoredPointClusterxp> avgLinkageClusterSet(std::vector<ColoredPointClusterxp> clusterSetIn, double distWeight, double colWeight, double distanceBound) {
	for (size_t i = 0; i < clusterSetIn.size (); i++) {
		size_t j = i + 1; 
		while (j < clusterSetIn.size ()) {
			if (clusterSetIn.at(i).avgClusterDistanceNorm(clusterSetIn.at(j), distWeight, colWeight) <= distanceBound) {
				clusterSetIn.at(i).mergeWithCluster(clusterSetIn.at(j)); //merge
				clusterSetIn.erase(clusterSetIn.begin()+j);  // ... delete, 
			}		
			else {
				j++;
			}	
		}
	}
	return clusterSetIn;
  }


/********************************************************************/
std::vector<ColoredPointClusterxp> singleLinkageClusterSet(std::vector<ColoredPointClusterxp> clusterSetIn, double distWeight, double colWeight, double distanceBound) {
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
	return clusterSetIn;
  }



/********************************************************************/

void erasePlaneCluster(std::vector<ColoredPointClusterxp>& clusterSet, double maxClusterLength) {
	for (size_t i = 0; i < clusterSet.size(); i++) {
		if (clusterSet.at(i).getMaxClusterLength() > maxClusterLength) {
			clusterSet.erase(clusterSet.begin() + i);
		}
	}
}


/********************************************************************/
void assignTrackingIds(std::vector<ColoredPointClusterxp>& curClusterSet, std::vector<ColoredPointClusterxp>& lastClusterSet) {
	for (int i = 0; i < 64; i++) {
	   trackingIdFlags[i] = false;
	}

	double currentNorm = 0.0;
	double minDist = 1000.0;
	size_t minIndex = 0;
        double minDistThres = 0.5;
	

	for (size_t i = 0; i < curClusterSet.size(); i++) {
		for (size_t j = 0; j < lastClusterSet.size(); j++) {
			currentNorm = curClusterSet.at(i).clusterDistanceNormForTracking(lastClusterSet.at(j), 1.0, 0.0);		
			minDist = 1000; minIndex = 0;			
			if (currentNorm < minDist) {
				minDist = currentNorm;
				minIndex = j;
			}
		}	   
		
		if (lastClusterSet.size() == 0) {
			break;
		}

		if (minDist < minDistThres) {  //found Greedy Match
			if (lastClusterSet.size() > minIndex) {
				curClusterSet.at(i).trackingId = lastClusterSet.at(minIndex).trackingId;
				lastClusterSet.erase(lastClusterSet.begin() + minIndex);
				trackingIdFlags[minIndex%64] = true;
			} else {
				cerr << " ERROR: lastClustersize: " << lastClusterSet.size() << " minIndex " << minIndex <<  std::endl;
			}
		}
	} 

	//cerr << " curClustersize: " << curClusterSet.size() << " lastClustersize: " << lastClusterSet.size() <<  std::endl;

	for (size_t i = 0; i < curClusterSet.size(); i++) {
		if (curClusterSet.at(i).trackingId == -1) {
			for (int j = 0; j < 64; j++) {
				if (trackingIdFlags[j] == false) {
					curClusterSet.at(i).trackingId = j;
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


void dumpOut(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointClusterxp>& clusterSet, std::vector<pcl::PointXYZRGB>& planeRgbdPixelSet, bool imageOnly) {
 
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

	for (size_t j = 0; j < planeRgbdPixelSet.size(); j++) {
	  	myfile << planeRgbdPixelSet.at(j).x << ","<< planeRgbdPixelSet.at(j).y <<","<< planeRgbdPixelSet.at(j).z <<","<< planeRgbdPixelSet.at(j).r << ","<< planeRgbdPixelSet.at(j).g <<","<< planeRgbdPixelSet.at(j).b <<","<< 0 <<"\n"; 
	    }

         

	 for (size_t i = 0; i < clusterSet.size(); i++) {

	   for (size_t j = 0; j < clusterSet.at(i).points.size(); j++) {
	  	myfile << clusterSet.at(i).points.at(j).x << ","<<clusterSet.at(i).points.at(j).y <<","<<clusterSet.at(i).points.at(j).z <<","<< clusterSet.at(i).points.at(j).r << ","<<clusterSet.at(i).points.at(j).g <<","<<clusterSet.at(i).points.at(j).b<<","<<i + 1<<"\n"; 
	    }
	}

        myfile.close();
	} else {
		strstr3 << "/home/goehring/ros_workspace/testbolt/ros/vision/kinect/" << seconds << "_debugImage.bmp";
		cv::imwrite(strstr3.str().c_str() , cv_ptr->image);
	}

}



/********************************************************************/

  void cloudSCb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
	//ROS_INFO("PCL-Data");
//input.header.frame_id
//

// 	transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)

	try{
		transformationWorked = pcl_ros::transformPointCloud("/base_link", *input, *cloud_transformed_ptr, listener);
	   }
	catch (tf::TransformException ex)
	    {
		ROS_INFO("Transform Exception in Kinect find cluster");	
	    }
	
    if (cycleCountPcl%100 == 0)	
	ROS_INFO(" new pcl data received ");

	  if (transformationWorked) {
	//	ROS_INFO(" Transformation worked ");
		  sor.setInputCloud (cloud_transformed_ptr); //USED TO BE INPUT
	  } else {
	//	ROS_INFO(" Transformation failed ");
		  sor.setInputCloud (input); //USED TO BE INPUT
	  }

	  sor.setLeafSize (VoxelizeLeafSize, VoxelizeLeafSize, VoxelizeLeafSize);
	  sor.filter (cloud_filtered);

          pcl::fromROSMsg (cloud_filtered, cloudRGB);
	  cycleCountPcl++;
  }


/********************************************************************/

  void imageSCb(const sensor_msgs::ImageConstPtr& msg)
  {

    std::vector<pcl::PointXYZRGB> rgbdPixelSet;
    std::vector<pcl::PointXYZRGB> planeRgbdPixelSet;

    if (cycleCountImg%100 == 0)	
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


	    pcl::fromROSMsg (cloud_filtered, cloudRGB);

	//copyPointCloud(cloud_xyz, cloud_xyzrgb);

	/* SAC Starts */

	  pcl::ModelCoefficients coefficients;
	  pcl::PointIndices inliers;
	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	  // Optional
	  seg.setOptimizeCoefficients (true);
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setDistanceThreshold (0.01); //0.01

	  seg.setInputCloud (cloudRGB.makeShared ());

	  seg.segment (inliers, coefficients);

	if (coefficients.values.size() > 3) {
			  coeffManual[0] = coefficients.values.at(0);
	  		  coeffManual[1] = coefficients.values.at(1);
	  		  coeffManual[2] = coefficients.values.at(2);
	  		  coeffManual[3] = coefficients.values.at(3);
	}

          reducePointCloudByDistanceAndHeight(cloudRGB, 1.5, 0.5);
	
	pcl::PointXYZRGB rgbdPixel;
	pcl::PointXYZRGB planeRgbdPixel;
	cv::Vec3b pixelColorVector;

	for (size_t i = 0; i < cloudRGB.points.size (); i++)
	  {

//ignore everything, matching the following criterion
		double signedPointDistanceToSACPlane = signedPointPlaneDistance(cloudRGB.points[i].x, cloudRGB.points[i].y, cloudRGB.points[i].z, coeffManual[0], coeffManual[1], coeffManual[2], coeffManual[3]);
	
	 if (cycleCountImg%100 == 0) {	
		ROS_INFO("A: %f B: %f C: %f D: %f Cyc %d", coeffManual[0], coeffManual[1], coeffManual[2], coeffManual[3], cycleCountImg);
	 }

//		if (true && (!((fabs(coeffManual[1]) > 0.75)))) {
//			break;	//ignore if table is not parallel to y,z plane 
//		}		//if points are on the table and cluster too - big --> paint it black


	if (cloudRGB.points[i].z < 1.5) {
		if (signedPointDistanceToSACPlane >= minDistanceAbovePlane) { // if points are on the table and a bit under - fill colorpointarray
			rgbdPixel.x = cloudRGB.points[i].x;
			rgbdPixel.y = cloudRGB.points[i].y;
			rgbdPixel.z = cloudRGB.points[i].z;
			rgbdPixel.r = cloudRGB.points[i].r;
			rgbdPixel.g = cloudRGB.points[i].g;
			rgbdPixel.b = cloudRGB.points[i].b;

	  	        rgbdPixelSet.push_back(rgbdPixel);

		} else 
		if (signedPointDistanceToSACPlane > minDistanceUnderPlane) {
				planeRgbdPixel.x = cloudRGB.points[i].x;
				planeRgbdPixel.y = cloudRGB.points[i].y;
				planeRgbdPixel.z = cloudRGB.points[i].z;
				planeRgbdPixel.r = cloudRGB.points[i].r;
				planeRgbdPixel.g = cloudRGB.points[i].g;
				planeRgbdPixel.b = cloudRGB.points[i].b;

			      planeRgbdPixelSet.push_back(planeRgbdPixel);

			}
		} else {
			continue;	//and if points not above the table 	
		}
	
	  } // for every Point in cff

	std::vector<ColoredPointClusterxp> clusterSet = createClusterSet(rgbdPixelSet);
	std::vector<ColoredPointClusterxp> planeClusterSet = createClusterSet(planeRgbdPixelSet);

	double stepSize = 0.02;
	double upperLimit = 0.2;
	for (double a = stepSize; ((a < upperLimit) && (clusterSet.size() >= 2)); a += stepSize) {	
		clusterSet = singleLinkageClusterSet(clusterSet, 2.0, 0.01, a);	
		planeClusterSet = singleLinkageClusterSet(planeClusterSet, 0.5, 0.01, a);	
	}


	if (captureNow) {
		dumpOut(cv_ptr, clusterSet, planeRgbdPixelSet, false);
		std::cerr << "Capturing ... Raw" << std::endl;
	}
	
//---------------------------------------------------------------A

	erasePlaneCluster(planeClusterSet, maxClusterLength);


	assignTrackingIds(clusterSet, lastClusterSet);

	assignTrackingIds(planeClusterSet, lastPlaneClusterSet);


        //siftExample(cv_ptr);

	if (!transformationWorked) {
		drawClusterPoints(cv_ptr, clusterSet, false);  //only above table
		drawClusterPoints(cv_ptr, planeClusterSet, true);  //on table
	}

	if (captureNow) {
		dumpOut(cv_ptr, clusterSet, planeRgbdPixelSet, true);
		std::cerr << "Capturing ... Debug" << std::endl;
		captureNow = false;	
	}

//---------------------------------------------------------------B

    sensor_msgs::PointCloud2 publishedClusterPC2;
    pcl::PointCloud<pcl::PointXYZRGB> publishCluster;
    pcl::PointXYZRGB helpPoint;

    geometry_msgs::Vector3 bestCluster;
	int maxClusterSize = 0;
	int maxClusterID = 0;

    //geometry_msgs::PoseArray poseArray;

    if (clusterSet.size() > 0) {
	for (size_t i = 0; i < clusterSet.size(); i++) {
		if ((int)clusterSet.at(i).points.size() > maxClusterSize) {
			maxClusterID = i;
			maxClusterSize = clusterSet.at(i).points.size();		
		}
	}
	bestCluster.x = clusterSet.at(maxClusterID).center.x;
	bestCluster.y = clusterSet.at(maxClusterID).center.y;
	bestCluster.z = clusterSet.at(maxClusterID).center.z;

	for (size_t j = 0; j < clusterSet.at(maxClusterID).points.size(); j++) {
		helpPoint.x = clusterSet.at(maxClusterID).points.at(j).x;
		helpPoint.y = clusterSet.at(maxClusterID).points.at(j).y;
		helpPoint.z = clusterSet.at(maxClusterID).points.at(j).z;

                helpPoint.r = clusterSet.at(maxClusterID).points.at(j).r;
                helpPoint.g = clusterSet.at(maxClusterID).points.at(j).g;
                helpPoint.b = clusterSet.at(maxClusterID).points.at(j).b;

		
		publishCluster.push_back(helpPoint);

	}
    } else {
	bestCluster.x = bestCluster.y = 0; bestCluster.z = 1;
    }
 
    publishedClusterPC2.header.frame_id = "/base_link";

    pcl::toROSMsg(publishCluster, publishedClusterPC2);
    cloud_percept.publish(publishedClusterPC2); // is still empty

    ROS_INFO(" Best Cluster: X: %f | Y: %f | Z: %f" ,bestCluster.x , bestCluster.y , bestCluster.z);
//	ROS_INFO("FOO");

    cloud_vector.publish(bestCluster);



    lastClusterSet = clusterSet;
    cycleCountImg++;
    image_pub_.publish(cv_ptr->toImageMsg());



  } //method end


/********************************************************************/

void drawClusterPoints(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointClusterxp>& clusterSet, bool isOnTable = false) {

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

		if (clusterSet.at(c).points.size() < 10) {   //minClusterSize
			continue;
		} 

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

     
////
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

