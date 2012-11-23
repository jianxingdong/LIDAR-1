/*
 * SensorSync.cpp
 *
 *  Created on: Nov 9, 2012
 *      Author: kent
 */

#include "SensorSync.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "SensorSynchronizer");

	SensorSync sensorSync;

	ros::spin();

	return 0;
}

SensorSync::SensorSync()
{
	this->nodeHandler = ros::NodeHandle();

	this->odometryPublisher = this->nodeHandler.advertise<nav_msgs::Odometry>("/LIDAR/synched/odometry", 10);
	this->imuPublisher = this->nodeHandler.advertise<sensor_msgs::Imu>("/LIDAR/synched/imu", 10);
	this->pointCloudPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud>("/LIDAR/synched/pointCloud", 10);

	this->odometrySubscriber = this->nodeHandler.subscribe("/LIDAR/simulatedOdometry", 10, &SensorSync::odometryCallback, this);
	this->imuSubscriber = this->nodeHandler.subscribe("/fmSensors/imu", 10, &SensorSync::imuCallback, this);
	this->pointCloudSubscriber = this->nodeHandler.subscribe("/LIDAR/pointCloud", 10, &SensorSync::pointCloudCallback, this);
}

SensorSync::~SensorSync()
{

}

void SensorSync::odometryCallback(const nav_msgs::Odometry::ConstPtr& data)
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

	this->synchronizeStamps();
}

void SensorSync::imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
	this->imu.header.frame_id = data.get()->header.frame_id;
	this->imu.orientation = data.get()->orientation;
}

void SensorSync::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& data)
{
	this->pointCloud.header.frame_id = data.get()->header.frame_id;
	this->pointCloud.channels = data.get()->channels;
	this->pointCloud.points = data.get()->points;
}

void SensorSync::synchronizeStamps(void)
{
	//	Update stamps
	this->imu.header.stamp = this->odometry.header.stamp;
	this->pointCloud.header.stamp = this->odometry.header.stamp;

	//	Publish
	this->odometryPublisher.publish(this->odometry);
	this->imuPublisher.publish(this->imu);
	this->pointCloudPublisher.publish(this->pointCloud);
}
