/*
 * TerrainMapper.cpp
 *
 *  Created on: 24 Nov 2012
 *      Author: kent
 */

#include "TerrainMapper.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "TerrainMapper");

	TerrainMapper tMappper;
	tMappper.makeItSpin();

	return 0;
}

TerrainMapper::TerrainMapper() : imageTransporter(this->nodeHandler)
{
	this->imagePublisher = this->imageTransporter.advertise("/LIDAR/terrainMap", 10);
	this->output.header.frame_id = "terrain_link";
	this->output.encoding = sensor_msgs::image_encodings::MONO8;

	//	Setup callbacks
	this->odometrySubscriber = this->nodeHandler.subscribe("/LIDAR/synched/odometry", 10, &TerrainMapper::odometryCallback, this);
	this->imuSubscriber = this->nodeHandler.subscribe("/LIDAR/synched/imu", 10, &TerrainMapper::imuCallback, this);
	this->pointCloudSubscriber = this->nodeHandler.subscribe("/LIDAR/synched/pointCloud", 10, &TerrainMapper::pointCloudCallback, this);

	//	Setup cell map
	//	***********************************************************
	//	x:	0.5m @ 0.02m	y:	1.0m @ 0.02m
	//	z:	-0.64m to 0.64m @ 0.005m
	//	***********************************************************
	this->depth 	= 0.5;		//	[m]
	this->xRes		= 0.02;		//	[m]
	this->width 	= 1.0;		//	[m]
	this->yRes		= 0.02;		//	[m]

	this->refColor	= 128;
	this->zRes		= 0.005;	//	[m]

	this->imageHeight	= (int)(depth / xRes);
	this->imageWidth 	= (int)(width / yRes);

	this->image = cv::Mat::zeros(this->imageHeight, this->imageWidth, CV_8UC1);

	this->resetMap();
	//	Setup cell map done
}

TerrainMapper::~TerrainMapper()
{

}

void TerrainMapper::makeItSpin (void)
{
	ros::Rate r(10);

	while (ros::ok())
	{
		ros::spinOnce();

		this->output.header.stamp = ros::Time::now();
		this->output.image = this->image;

		this->imagePublisher.publish(this->output.toImageMsg());
	}
}

void TerrainMapper::resetMap (void)
{
	for (int i = 0; i < this->imageHeight; i++) for (int j = 0; j < this->imageWidth; j++)
		this->image.at<unsigned char>(i, j) = (unsigned char)this->refColor;
}

void TerrainMapper::odometryCallback(const nav_msgs::Odometry::ConstPtr& data)
{

}

void TerrainMapper::imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{

}

void TerrainMapper::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& data)
{

}
