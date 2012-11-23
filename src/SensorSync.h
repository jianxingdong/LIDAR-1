/*
 * SensorSync.h
 *
 *  Created on: Nov 9, 2012
 *      Author: kent
 */

#ifndef SENSORSYNC_H_
#define SENSORSYNC_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>

class SensorSync
{
private:
	ros::NodeHandle nodeHandler;
	ros::Publisher odometryPublisher;
	ros::Publisher imuPublisher;
	ros::Publisher pointCloudPublisher;
	ros::Subscriber odometrySubscriber;
	ros::Subscriber imuSubscriber;
	ros::Subscriber pointCloudSubscriber;

	nav_msgs::Odometry odometry;
	sensor_msgs::Imu imu;
	sensor_msgs::PointCloud pointCloud;

	void odometryCallback (const nav_msgs::Odometry::ConstPtr& data);
	void imuCallback (const sensor_msgs::Imu::ConstPtr& data);
	void pointCloudCallback (const sensor_msgs::PointCloud::ConstPtr& data);
	void synchronizeStamps (void);

public:
	SensorSync();
	virtual ~SensorSync();
};

#endif /* SENSORSYNC_H_ */
