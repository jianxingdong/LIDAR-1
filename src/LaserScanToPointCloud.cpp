/*
 * LaserScanToPointCloud.cpp
 *
 *  Created on: Nov 12, 2012
 *      Author: kent
 */

#include "LaserScanToPointCloud.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "OdometrySimulator");

	LaserScanToPointCloud laserScanToPointCloud = LaserScanToPointCloud();

	ros::spin();

	return 0;
}

LaserScanToPointCloud::LaserScanToPointCloud()
{
	this->nodeHandle = ros::NodeHandle();
	this->pointCloudPublisher = this->nodeHandle.advertise<sensor_msgs::PointCloud>("/LIDAR/pointCloud", 10);
	this->laserScanSubscriber = this->nodeHandle.subscribe("/LIDAR/synched/laserScan", 10, &LaserScanToPointCloud::laserScanCallback, this);

	this->laserNotifier = tf::MessageFilter<sensor_msgs::LaserScan>(this->laserScanSubscriber, this->transformListener, "base_link", 10);
	this->laserNotifier.registerCallback(boost::bind(&LaserScanToPointCloud::laserScanCallback, this, _1));
	this->laserNotifier.setTolerance(ros::Duration(0.1));
}

LaserScanToPointCloud::~LaserScanToPointCloud(){}

void LaserScanToPointCloud::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
	try
	{
		this->laserProjection.transformLaserScanToPointCloud("base_link", *laserScan, this->pointCloud, this->transformListener);
	}
	catch (tf::TransformException& e)
	{
		ROS_WARN("Laser scanner callback generated exception: %s", e.what());
	}

	this->pointCloudPublisher.publish(this->pointCloud);
}
