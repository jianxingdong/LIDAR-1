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
	this->pointCloudSubscriber = this->nodeHandler.subscribe("/LIDAR/pointCloud", 10, &TerrainMapper::pointCloudCallback, this);

	//	Setup cell map
	//	***********************************************************
	//	x:	0.5m @ 0.02m	y:	1.0m @ 0.02m
	//	z:	-0.64m to 0.64m @ 0.005m
	//	***********************************************************
	this->depth 	= 0.5;		//	[m]
	this->xRes		= 0.02;		//	[m]
	this->width 	= 0.8;		//	[m]
	this->yRes		= 0.02;		//	[m]

	this->refColor	= 128;
	this->zRes		= 0.002;	//	[m]

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

unsigned char TerrainMapper::getPixel (int x, int y)
{
	if (x >= 0 && x < this->imageHeight &&	y >= 0 && y < this->imageWidth)
		return this->image.at<unsigned char>(x, y);

	return 0;
}

void TerrainMapper::setPixel (int x, int y, unsigned char color)
{
	if (x >= 0 && x < this->imageHeight &&	y >= 0 && y < this->imageWidth)
		this->image.at<unsigned char>(x, y) = color;
}

void TerrainMapper::pointCloudToMap (void)
{
	this->resetMap();

	double x, y, z;
	int currentX, currentY;
	unsigned char color;

	for (unsigned int i = 0; i < this->pointCloud.points.size(); i++)
	{
		x = (double)this->pointCloud.points.at(i).x;
		y = (double)this->pointCloud.points.at(i).y;
		z = (double)this->pointCloud.points.at(i).z;

		//currentX = this->imageHeight / 5; // (int)(x / this->xRes);
		currentX = (int)(x / this->xRes);
		currentY = (int)((y + (this->width / 2)) / this->yRes);
		color = (unsigned char)(((double)this->getPixel(currentX, currentY) + (this->refColor + (z / this->zRes))) / 2);
		this->setPixel(currentX, currentY, color);
	}
}

void TerrainMapper::odometryToMap (void)
{
	//
}

void TerrainMapper::odometryCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	this->odometry.header.frame_id = data.get()->header.frame_id;
	this->odometry.header.stamp = data.get()->header.stamp;
	this->odometry.pose.pose.orientation = data.get()->pose.pose.orientation;
	this->odometry.pose.pose.position.x = data.get()->pose.pose.position.x;
	this->odometry.pose.pose.position.y = data.get()->pose.pose.position.y;
	this->odometry.pose.pose.position.z = data.get()->pose.pose.position.z;

	this->odometry.twist.twist.angular.x = data.get()->twist.twist.angular.x;
	this->odometry.twist.twist.angular.y = data.get()->twist.twist.angular.y;
	this->odometry.twist.twist.angular.z = data.get()->twist.twist.angular.z;
	this->odometry.twist.twist.linear.x = data.get()->twist.twist.linear.x;
	this->odometry.twist.twist.linear.y = data.get()->twist.twist.linear.y;
	this->odometry.twist.twist.linear.z = data.get()->twist.twist.linear.z;

	this->odometryToMap();
}

void TerrainMapper::imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
	this->imu.header.frame_id = data.get()->header.frame_id;
	this->imu.orientation = data.get()->orientation;
	this->imu.orientation_covariance = data.get()->orientation_covariance;
}

void TerrainMapper::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& data)
{
	this->pointCloud.header.frame_id = data.get()->header.frame_id;
	this->pointCloud.channels = data.get()->channels;
	this->pointCloud.points = data.get()->points;

	this->pointCloudToMap();
}
