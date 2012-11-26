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

class TerrainMapper
{
private:
	ros::NodeHandle nodeHandler;
	image_transport::Publisher imagePublisher;

	//	OpenCV bridge to ROS
	image_transport::ImageTransport imageTransporter;
	cv_bridge::CvImage output;

	//	Cell map
	float depth, width, cellResolution;		//	x-y dimension
	float referenceColor, zResolution;		//	z dimension
	int maxX, maxY;
	cv::Mat image;

	void resetMap (void);

public:
	TerrainMapper();
	virtual ~TerrainMapper();

	void makeItSpin (void);
};

#endif /* TERRAINMAPPER_H_ */
