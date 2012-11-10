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
#include <sensor_msgs/LaserScan.h>

class SensorSync
{
private:
	ros::NodeHandle nodeHandler;
	ros::Publisher odometryPublisher;
	ros::Publisher imuPublisher;
	ros::Publisher laserScanPublisher;
	ros::Subscriber odometrySubscriber;
	ros::Subscriber imuSubscriber;
	ros::Subscriber laserScanSubscriber;

	nav_msgs::Odometry odometry;
	sensor_msgs::Imu imu;
	sensor_msgs::LaserScan laserScan;

	void odometryCallback (const nav_msgs::Odometry::ConstPtr& data);
	void imuCallback (const sensor_msgs::Imu::ConstPtr& data);
	void laserScanCallback (const sensor_msgs::LaserScan::ConstPtr& data);
	void synchronizeStamps (void);
	void publish (void);

public:
	SensorSync();
	virtual ~SensorSync();
};

#endif /* SENSORSYNC_H_ */
