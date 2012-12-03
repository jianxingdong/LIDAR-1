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
	this->laserScanSubscriber.subscribe(this->nodeHandler, "/SICK/scan", 10);
	this->laserNotifier = new tf::MessageFilter<sensor_msgs::LaserScan>(this->laserScanSubscriber, this->transformListener, "/base_link", 10);
	this->laserNotifier->registerCallback(boost::bind(&ScanToPointCloud::laserScanCallback, this, _1));
	this->laserNotifier->setTolerance(ros::Duration(0.1));
}

ScanToPointCloud::~ScanToPointCloud()
{

}

void ScanToPointCloud::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
	try
	{
		this->laserProjection.transformLaserScanToPointCloud("/terrain_link", *data, this->pointCloud, this->transformListener);
	}
	catch (tf::TransformException& e)
	{
		ROS_WARN("Laser scanner callback generated exception: %s", e.what());
	}

	this->pointCloudPublisher.publish(this->pointCloud);
}
