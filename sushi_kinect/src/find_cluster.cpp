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

/////////////

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include "ColoredPointCluster.h"
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
  double oldManual[4];
  bool initialRun;

  ros::Subscriber cloud_sub;
  ros::Publisher cloud_pub;

  ros::Subscriber capture_sub;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::NodeHandle n;
  ros::Publisher cloud_percept; 
  //ros::NodeHandle pa;
  //ros::Publisher pose_array; 

  cv_bridge::CvImagePtr cv_ptr;

   static const double OpeningAngleHorizontal = 34.0 / 180.0 * 3.14159266;
   static const double OpeningAngleVertical = 27.0 / 180.0 * 3.14159266;
   static const int ImageVerticalOffset = 20;
   static const int ImageHorizontalOffset = -5;

   static const double VoxelizeLeafSize = 0.02;
   static const double maxClusterLength = 0.4;
   static const double minDistanceAbovePlane = -0.0125;
   static const double doRANSAC = true; 


  //Store point cloud data
  sensor_msgs::PointCloud2 cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ> cff;
  //pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;

  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;

  int cycleCountPcl;
  int cycleCountImg;

  bool trackingIdFlags[64];
  std::vector<ColoredPointCluster> lastClusterSet;

  bool captureNow;
  //bool doRANSAC;

public:


  FindCluster()
    : it_(nh_)
  {
    capture_sub = cp.subscribe("/bolt/vision/capture", 1, &FindCluster::captureThis, this);
    cloud_sub = pt_.subscribe("cloud_in", 1, &FindCluster::cloudSCb, this);
    image_sub_ = it_.subscribe("image_in", 1, &FindCluster::imageSCb, this);
    image_pub_ = it_.advertise("/bolt/vision/image", 1);
    
    cloud_percept = n.advertise<sensor_msgs::PointCloud2>("/bolt/vision/biggest_cloud_cluster", 10);
    //pose_array = pa.advertise<geometry_msgs::PoseArray>("/bolt/vision/pose_array", 10);


    cycleCountPcl = 0;
    cycleCountImg = 0;

    captureNow = false;
    initialRun = true;
    oldManual[0] = 0.0;
    oldManual[1] = 0.0;
    oldManual[2] = 0.0;
    oldManual[3] = 0.0;
  }

  ~FindCluster()
  {
  }





  void reducePointCloudByDistanceAndHeight(pcl::PointCloud<pcl::PointXYZ>& cloud, double distanceBoundary, double heightBoundary) {

				
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

  void cloudSCb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    if (cycleCountPcl%100 == 0)	
	std::cerr << " new pcl data received " << std::endl;

 //   sensor_msgs::PointCloud2 reducedCloud;
 //   sensor_msgs::PointCloud2Ptr reducedCloudPtr;

//    pcl::fromROSMsg (*input, cff);



 //   pcl::toROSMsg (oldFormatCloud, reducedCloud);
 //   *reducedCloudPtr = reducedCloud;



//---- Voxelization
	  sor.setInputCloud (input);
	  sor.setLeafSize (VoxelizeLeafSize, VoxelizeLeafSize, VoxelizeLeafSize);
	  sor.filter (cloud_filtered);
	  cycleCountPcl++;

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
  std::vector<ColoredPointCluster> createClusterSet(std::vector<ColoredPoint> coloredPointSet) {
	std::vector<ColoredPointCluster> clusterSet;
	ColoredPointCluster cluster;

	for (size_t i = 0; i < coloredPointSet.size (); i++) {
		cluster.clear();
		cluster.addElement(coloredPointSet.at(i)); 
		//if (i % 100 == 0) {std::cerr << " createClusterSet, i " << i << " x: " << coloredPointSet.at(i).x << std::endl;}

		clusterSet.push_back(cluster);
	}
	return clusterSet;
  }


/********************************************************************/
std::vector<ColoredPointCluster> avgLinkageClusterSet(std::vector<ColoredPointCluster> clusterSetIn, double distWeight, double colWeight, double distanceBound) {
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
std::vector<ColoredPointCluster> singleLinkageClusterSet(std::vector<ColoredPointCluster> clusterSetIn, double distWeight, double colWeight, double distanceBound) {
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

void erasePlaneCluster(std::vector<ColoredPointCluster>&  clusterSet, double coefficients[], double maxClusterLength, double minDistanceAbovePlane) {
	
	

	for (size_t i = 0; i < clusterSet.size(); i++) {
		if ((fabs(signedPointPlaneDistance(clusterSet.at(i).center.x, clusterSet.at(i).center.y, clusterSet.at(i).center.z, coefficients[0], coefficients[1], coefficients[2], coefficients[3])) < minDistanceAbovePlane) && (clusterSet.at(i).getMaxClusterLength() > maxClusterLength)) {
			clusterSet.erase(clusterSet.begin() + i);
		}
	}
}


/********************************************************************/
void assignTrackingIds(std::vector<ColoredPointCluster>& curClusterSet, std::vector<ColoredPointCluster>& lastClusterSet) {
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


void dumpOut(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointCluster>& clusterSet, std::vector<ColoredPoint>& planeRgbdPixelSet, bool imageOnly) {
 
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

  void imageSCb(const sensor_msgs::ImageConstPtr& msg)
  {
    std::vector<ColoredPoint> rgbdPixelSet;
    std::vector<ColoredPoint> planeRgbdPixelSet;

    if (cycleCountImg%100 == 0)	
    std::cerr << " new image data received " << std::endl;

    
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


	    pcl::fromROSMsg (cloud_filtered, cff);

	//copyPointCloud(cloud_xyz, cloud_xyzrgb);

	/* SAC Starts */

	  pcl::ModelCoefficients coefficients;
	  pcl::PointIndices inliers;
	  // Create the segmentation object
	  pcl::SACSegmentation<pcl::PointXYZ> seg;
	  // Optional
	  seg.setOptimizeCoefficients (true);
	  // Mandatory
	  seg.setModelType (pcl::SACMODEL_PLANE);
	  seg.setMethodType (pcl::SAC_RANSAC);
	  seg.setDistanceThreshold (0.01); //0.01

	  seg.setInputCloud (cff.makeShared ());

//	   private_node_handle_A.param<double>("/bolt/vision/plane_Param_A", paramA, 0.0);
//	   private_node_handle_B.param<double>("/bolt/vision/plane_Param_B", paramB, 1.0);
//	   private_node_handle_C.param<double>("/bolt/vision/plane_Param_C", paramC, 0.0);
	   private_node_handle_D.param<double>("/bolt/vision/plane_Param_D", paramD, 0.0);
	   private_node_handle_X.param<double>("/bolt/vision/plane_Param_X", paramX, 0.0);



	  if (doRANSAC) {
			  seg.segment (inliers, coefficients);
			  coeffManual[0] = coefficients.values.at(0);
	  		  coeffManual[1] = coefficients.values.at(1);
	  		  coeffManual[2] = coefficients.values.at(2);
	  		  coeffManual[3] = coefficients.values.at(3);
		if (!initialRun) {
			if (!(fabs(coeffManual[1]) > 0.75)) {
				coeffManual[0] = oldManual[0];
				coeffManual[1] = oldManual[1];
				coeffManual[2] = oldManual[2];
				coeffManual[3] = oldManual[3];
			} else {
			  oldManual[0] = coeffManual[0];
			  oldManual[1] = coeffManual[1];
			  oldManual[2] = coeffManual[2];
			  oldManual[3] = coeffManual[3];
			}
		}


	  } else {
//	  	coefficients.values.at(0) = paramA;
//	  	coefficients.values.at(1) = paramB;
//	  	coefficients.values.at(2) = paramC;
	  	coeffManual[0] = 0.0;
	  	coeffManual[1] = cos(paramX / 180.0 * 3.141593);
	  	coeffManual[2] = sin(paramX / 180.0 * 3.141593);
	  	coeffManual[3] = paramD;
		//cerr << " A: " << coefficients.values.at(0) << " B: " << coefficients.values.at(1) << " C: " << coefficients.values.at(2) << " D: " << coefficients.values.at(3) << std::endl; 
	  }


          reducePointCloudByDistanceAndHeight(cff, 1.5, 0.5);
	
	ColoredPoint rgbdPixel;
	ColoredPoint planeRgbdPixel;
	cv::Vec3b pixelColorVector;

	for (size_t i = 0; i < cff.points.size (); i++)
	  {

//ignore everything, matching the following criterion
		double signedPointDistanceToSACPlane = signedPointPlaneDistance(cff.points[i].x, cff.points[i].y, cff.points[i].z, coeffManual[0], coeffManual[1], coeffManual[2], coeffManual[3]);
		

		if (doRANSAC && (!((fabs(coeffManual[1]) > 0.75)))) {
			break;	//ignore if table is not parallel to y,z plane 
		}		//if points are on the table and cluster too - big --> paint it black
		if (!((signedPointDistanceToSACPlane < minDistanceAbovePlane) || (cff.points[i].z > 1.5))) { // if points are on the table and a bit under - fill colorpointarray
			rgbdPixel.x = cff.points[i].x;
			rgbdPixel.y = cff.points[i].y;
			rgbdPixel.z = cff.points[i].z;
		} else {
			if (fabs(signedPointDistanceToSACPlane) < minDistanceAbovePlane) {
				planeRgbdPixel.x = cff.points[i].x;
				planeRgbdPixel.y = cff.points[i].y;
				planeRgbdPixel.z = cff.points[i].z;

				int pix_x = 0;
				int pix_y = 0;	     
		
			  	calculateImagePositionFrom3dPoint(cff.points[i].x, cff.points[i].y, cff.points[i].z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x, pix_y);
		
			      if ((pix_x >= 0) && (pix_x < cv_ptr->image.cols) && (pix_y >= 0) && (pix_y < cv_ptr->image.rows)) {
			      cv::Point pointColorComponents(pix_x, pix_y);
		
			      pixelColorVector = cv_ptr->image.at<cv::Vec3b>(pointColorComponents);
			      planeRgbdPixel.r = pixelColorVector[2];
			      planeRgbdPixel.g = pixelColorVector[1];
			      planeRgbdPixel.b = pixelColorVector[0];
			      } else {
				planeRgbdPixel.r = 0;
  			        planeRgbdPixel.g = 0;
			        planeRgbdPixel.b = 0;
			      }
	
			      planeRgbdPixelSet.push_back(planeRgbdPixel);

			}
			continue;	//and if points not above the table 	
		}
		
		int pix_x = 0;
		int pix_y = 0;	     
	
	  	calculateImagePositionFrom3dPoint(cff.points[i].x, cff.points[i].y, cff.points[i].z, cv_ptr->image.cols, cv_ptr->image.rows, OpeningAngleHorizontal, OpeningAngleVertical, pix_x, pix_y);
	     if (isPointWithinBoundaries(0, cv_ptr->image.cols, 0, cv_ptr->image.rows, 0, 0, pix_x, pix_y)) {
		
	      cv::Point pointColorComponents(pix_x, pix_y);
		
	      pixelColorVector = cv_ptr->image.at<cv::Vec3b>(pointColorComponents);
	      rgbdPixel.r = pixelColorVector[2];
	      rgbdPixel.g = pixelColorVector[1];
	      rgbdPixel.b = pixelColorVector[0];
	      } else {
		planeRgbdPixel.r = 0;
  	        planeRgbdPixel.g = 0;
	        planeRgbdPixel.b = 0;
	      }
	
	      rgbdPixelSet.push_back(rgbdPixel);


	  } // for every Point in cff


	std::vector<ColoredPointCluster> clusterSet = createClusterSet(rgbdPixelSet);

	double stepSize = 0.02;
	double upperLimit = 0.2;
	for (double a = stepSize; ((a < upperLimit) && (clusterSet.size() >= 2)); a += stepSize) {	
		clusterSet = singleLinkageClusterSet(clusterSet, 2.0, 0.01, a);	
	}


	if (captureNow) {
		dumpOut(cv_ptr, clusterSet, planeRgbdPixelSet, false);
		std::cerr << "Capturing ... Raw" << std::endl;
	}
	
//---------------------------------------------------------------A

	erasePlaneCluster(clusterSet, coeffManual, maxClusterLength, minDistanceAbovePlane);


	assignTrackingIds(clusterSet, lastClusterSet);


        //siftExample(cv_ptr);


	drawClusterPoints(cv_ptr, clusterSet);

	if (captureNow) {
		dumpOut(cv_ptr, clusterSet, planeRgbdPixelSet, true);
		std::cerr << "Capturing ... Debug" << std::endl;
		captureNow = false;	
	}

//---------------------------------------------------------------B

    sensor_msgs::PointCloud2 publishedCluster;
    pcl::PointCloud<pcl::PointXYZ> publishClusterXYZ;
    pcl::PointXYZ helpPoint;

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

		
		publishClusterXYZ.push_back(helpPoint);

	}
    } else {
	bestCluster.x = bestCluster.y = 0; bestCluster.z = 1;
    }


 

   //std::cerr << "A "  << paramA << std::endl;
   //std::cerr << "B "  << paramB << std::endl;
   //std::cerr << "C "  << paramC << std::endl;
   //std::cerr << "D "  << paramD << std::endl;

    initialRun = false;

    //cloud_percept.publish(bestCluster);




    cloud_percept.publish(publishedCluster); // is still empty



    lastClusterSet = clusterSet;
    cycleCountImg++;
    image_pub_.publish(cv_ptr->toImageMsg());



  } //method end


/********************************************************************/

void drawClusterPoints(cv_bridge::CvImagePtr& cv_ptr, std::vector<ColoredPointCluster>& clusterSet) {

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
  		      cv::rectangle(cv_ptr->image, pointPx0y0z0, pointPx1y1z0, CV_RGB(255, 0, 0));
  		      cv::rectangle(cv_ptr->image, pointPx0y0z1, pointPx1y1z1, CV_RGB(0, 0, 0));
		      cv::line(cv_ptr->image, pointPx0y0z0, pointPx0y0z1, CV_RGB(127, 0, 0));
		      cv::line(cv_ptr->image, pointPx1y1z0, pointPx1y1z1, CV_RGB(127, 0, 0));
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

