/*
 * LaserScanToPointCloud.h
 *
 *  Created on: Nov 12, 2012
 *      Author: kent
 */

#ifndef LASERSCANTOPOINTCLOUD_H_
#define LASERSCANTOPOINTCLOUD_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>

class ScanToPointCloud
{
private:
	ros::NodeHandle nodeHandler;
	ros::Publisher pointCloudPublisher;
	ros::Subscriber laserScanSubscriber;

	tf::TransformListener transformListener;
	laser_geometry::LaserProjection laserProjection;
	sensor_msgs::PointCloud pointCloud;

	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);

public:
	ScanToPointCloud();
	virtual ~ScanToPointCloud();
};

#endif /* LASERSCANTOPOINTCLOUD_H_ */
