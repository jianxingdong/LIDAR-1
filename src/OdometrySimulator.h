/*
 * OdometrySimulator.h
 *
 *  Created on: Nov 8, 2012
 *      Author: kent
 */

#ifndef ODOMETRYSIMULATOR_H_
#define ODOMETRYSIMULATOR_H_

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class OdometrySimulator
{
private:
	struct
	{
		double lX;			//	[m/s]
		double lY;			//	[m/s]
		double aZ;			//	[rad/s]
	} baseLinkTwist;

	struct
	{
		double x;			//	[m]
		double y;			//	[m]
		double yaw;			//	[rad]
	} odometryPose;

	ros::NodeHandle nodeHandler;
	ros::Publisher odometryPublisher;
	ros::Subscriber keyboardEventSubscriber;

	ros::Time currentTime;
	ros::Duration sampleTime;
	nav_msgs::Odometry odometry;
	geometry_msgs::TransformStamped stampedTransform;
	tf::TransformBroadcaster transformBroadcaster;

	void keyboardEventCallback (const std_msgs::Char::ConstPtr& data);
	void update(void);
	void publish(void);

public:
	OdometrySimulator();
	virtual ~OdometrySimulator();

	void makeItSpin(void);
};

#endif /* ODOMETRYSIMULATOR_H_ */
