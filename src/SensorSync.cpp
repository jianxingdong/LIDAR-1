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

	SensorSync sensorSync = SensorSync();

	ros::spin();

	return 0;
}

SensorSync::SensorSync()
{
	this->nodeHandler = ros::NodeHandle();

	this->odometryPublisher = this->nodeHandler.advertise<nav_msgs::Odometry>("/LIDAR/synched/odometry", 10);
	this->imuPublisher = this->nodeHandler.advertise<sensor_msgs::Imu>("/LIDAR/synched/imu", 10);
	this->laserScanPublisher = this->nodeHandler.advertise<sensor_msgs::LaserScan>("/LIDAR/synched/laserScan", 10);

	this->odometrySubscriber = this->nodeHandler.subscribe("/LIDAR/simulatedOdometry", 10, &SensorSync::odometryCallback, this);
	this->imuSubscriber = this->nodeHandler.subscribe("/LIDAR/IMU", 10, &SensorSync::imuCallback, this);
	this->laserScanSubscriber = this->nodeHandler.subscribe("/LIDAR/LASERSCAN", 10, &SensorSync::laserScanCallback, this);
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

void SensorSync::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	this->laserScan.header.frame_id = data.get()->header.frame_id;
	this->laserScan.angle_min = data.get()->angle_min;
	this->laserScan.angle_max = data.get()->angle_max;
	this->laserScan.angle_increment = data.get()->angle_increment;
	this->laserScan.time_increment = data.get()->time_increment;
	this->laserScan.scan_time = data.get()->scan_time;
	this->laserScan.range_min = data.get()->range_min;
	this->laserScan.range_max = data.get()->range_max;
	this->laserScan.ranges = data.get()->ranges;
	this->laserScan.intensities = data.get()->intensities;
}

void SensorSync::synchronizeStamps(void)
{
	//	Update stamps
	this->imu.header.stamp = this->odometry.header.stamp;
	this->laserScan.header.stamp = this->odometry.header.stamp;

	//	Publish
	this->odometryPublisher.publish(this->odometry);
	this->imuPublisher.publish(this->imu);
	this->laserScanPublisher.publish(this->laserScan);
}
