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
	double  depth, 		xRes;	//	x-dimension
	double	width, 		yRes;	//	y-dimension
	double	refColor,	zRes;	//	z-dimension
	int imageHeight,	xOffset;
	int imageWidth, 	yOffset;

	void resetMap (void);
	void odometryCallback (const nav_msgs::Odometry::ConstPtr& data);
	void imuCallback (const sensor_msgs::Imu::ConstPtr& data);
	void pointCloudCallback (const sensor_msgs::PointCloud::ConstPtr& data);

public:
	TerrainMapper();
	virtual ~TerrainMapper();

	void makeItSpin (void);
};

#endif /* TERRAINMAPPER_H_ */
