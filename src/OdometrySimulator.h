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
	ros::NodeHandle nodeHandler;
	ros::Publisher odometryPublisher;
	ros::Subscriber twistSubscriber;

	ros::Time oldTime;
	geometry_msgs::TwistStamped twist;
	nav_msgs::Odometry odometry;
	geometry_msgs::TransformStamped stampedTransform;
	tf::TransformBroadcaster transformBroadcaster;

	void twistCallback (const geometry_msgs::TwistStamped::ConstPtr& data);
	void update(double dt);
	void publish(void);

public:
	OdometrySimulator();
	virtual ~OdometrySimulator();

	void makeItSpin(void);
};

#endif /* ODOMETRYSIMULATOR_H_ */
