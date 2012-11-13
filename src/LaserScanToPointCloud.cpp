/*
 * LaserScanToPointCloud.cpp
 *
 *  Created on: Nov 12, 2012
 *      Author: kent
 */

#include "LaserScanToPointCloud.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "LaserScanTransformer");

	ScanToPointCloud laserScanToPointCloud;

	ros::spin();

	return 0;
}

ScanToPointCloud::ScanToPointCloud()
{
	this->nodeHandler = ros::NodeHandle();
	this->pointCloudPublisher = this->nodeHandler.advertise<sensor_msgs::PointCloud>("/LIDAR/pointCloud", 10);
	this->laserScanSubscriber = this->nodeHandler.subscribe("/LIDAR/synched/laserScan", 10, &ScanToPointCloud::laserScanCallback, this);
	this->transformListener.setExtrapolationLimit(ros::Duration(0.1));
}

ScanToPointCloud::~ScanToPointCloud()
{

}

void ScanToPointCloud::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	try
	{
		this->laserProjection.transformLaserScanToPointCloud("/base_link", *data, this->pointCloud, this->transformListener);
	}
	catch (tf::TransformException& e)
	{
		ROS_WARN("Laser scanner callback generated exception: %s", e.what());
	}

	this->pointCloudPublisher.publish(this->pointCloud);
}
