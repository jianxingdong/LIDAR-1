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
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <boost/bind.hpp>

class ScanToPointCloud
{
private:
	ros::NodeHandle nodeHandler;
	ros::Publisher pointCloudPublisher;
	message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSubscriber;
	tf::MessageFilter<sensor_msgs::LaserScan> *laserNotifier;

	tf::TransformListener transformListener;
	laser_geometry::LaserProjection laserProjection;
	sensor_msgs::PointCloud pointCloud;

	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);

public:
	ScanToPointCloud();
	virtual ~ScanToPointCloud();
};

#endif /* LASERSCANTOPOINTCLOUD_H_ */
