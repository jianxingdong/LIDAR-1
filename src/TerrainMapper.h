/*
 * TerrainMapper.h
 *
 *  Created on: 24 Nov 2012
 *      Author: kent
 */

#ifndef TERRAINMAPPER_H_
#define TERRAINMAPPER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl-1.5/pcl/point_cloud.h>
#include <pcl-1.5/pcl/point_types.h>

#include "Filters.h"

class TerrainMapper
{
private:
	ros::NodeHandle nodeHandler;
	ros::Subscriber odometrySubscriber;
	ros::Subscriber imuSubscriber;
	ros::Subscriber pointCloudSubscriber;
	image_transport::Publisher imagePublisher;

	//	OpenCV bridge to ROS
	image_transport::ImageTransport imageTransporter;
	cv_bridge::CvImage output;

	//	Cell map
	cv::Mat image;
	cv::Mat tempImage;
	double  depth, 		xRes;		//	x-dimension
	double	width, 		yRes;		//	y-dimension
	double	refColor,	zRes;		//	z-dimension
	double  minHeight, maxHeight;
	int imageHeight, imageWidth;

	//	Filter
	cv::Mat filterWindow;
	double filterWidth, filterHeight;

	//	Update data
	ros::Time previousTime;
	double linearDistanceMoved;
	//double angularDistanceMoved;
	sensor_msgs::PointCloud pointCloud;
	sensor_msgs::Imu imu;
	nav_msgs::Odometry odometry;

	//	Callbacks
	void odometryCallback (const nav_msgs::Odometry::ConstPtr& data);
	void imuCallback (const sensor_msgs::Imu::ConstPtr& data);
	void pointCloudCallback (const sensor_msgs::PointCloud::ConstPtr& data);

	//	Map functions
	unsigned char getPixel (int x, int y, cv::Mat *img);
	void setPixel (int x, int y, unsigned char color, cv::Mat *img);
	void resetMap (cv::Mat *img);
	void odometryToMap (void);
	void pointCloudToMap (void);

public:
	TerrainMapper();
	virtual ~TerrainMapper();

	void makeItSpin (void);
};

#endif /* TERRAINMAPPER_H_ */
