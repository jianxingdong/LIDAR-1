/*
 * OdometrySimulator.cpp
 *
 *  Created on: Nov 8, 2012
 *      Author: kent
 */

#include "OdometrySimulator.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "OdometrySimulator");

	OdometrySimulator odomSim;
	//odomSim.makeItSpin();
	ros::spin();

	return 0;
}

OdometrySimulator::OdometrySimulator()
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle();
	this->odometryPublisher = this->nodeHandler.advertise<nav_msgs::Odometry>("/LIDAR/simulatedOdometry", 10);
	this->keyboardEventSubscriber = this->nodeHandler.subscribe("/fmHMI/keyboardEventListener", 10, &OdometrySimulator::keyboardEventCallback, this);

	//	Base link twist essentials
	this->baseLinkTwist.lX = 0.1f;
	this->baseLinkTwist.lY = 0.0f;
	this->baseLinkTwist.aZ = 0.0f;

	//	Odometry state
	this->odometryPose.x =
	this->odometryPose.y = 0.0f;
	this->odometryPose.yaw = M_PI_4;

	//	Set currettime and set period
	this->currentTime = ros::Time::now();
	this->sampleTime = ros::Duration(0.1f);

	//	Initialize transform
	this->stampedTransform.header.frame_id = "/odom";
	this->stampedTransform.child_frame_id = "/base_link";
	this->stampedTransform.header.stamp = this->currentTime;

	this->stampedTransform.transform.rotation = tf::createQuaternionMsgFromYaw(this->odometryPose.yaw);
	this->stampedTransform.transform.translation.x = this->odometryPose.x;
	this->stampedTransform.transform.translation.y = this->odometryPose.y;
	this->stampedTransform.transform.translation.z = 0.0f;

	//	Initialize odometry
	this->odometry.header.frame_id = "/odom";
	this->odometry.child_frame_id = "/base_link";
	this->odometry.header.stamp = this->currentTime;

	this->odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->odometryPose.yaw);
	this->odometry.pose.pose.position.x = this->odometryPose.x;
	this->odometry.pose.pose.position.y = this->odometryPose.y;
	this->odometry.pose.pose.position.z = 0.0f;

	this->odometry.twist.twist.angular.z =
	this->odometry.twist.twist.linear.x =
	this->odometry.twist.twist.linear.y = 0.0f;
}

OdometrySimulator::~OdometrySimulator()
{

}

void OdometrySimulator::keyboardEventCallback(const std_msgs::Char::ConstPtr& data)
{
	if (data.get()->data == 0x20) 	//	Space-bar
	{
		this->update();
	}
}

void OdometrySimulator::update(ros::Duration delta_time)
{
	//	Calculate odometry
	double dt = delta_time;
	double deltaYaw = this->baseLinkTwist.aZ * dt;
	double deltaX = (this->baseLinkTwist.lX * cos(this->odometryPose.yaw) - this->baseLinkTwist.lY * sin(this->odometryPose.yaw)) * dt;
	double deltaY = (this->baseLinkTwist.lX * sin(this->odometryPose.yaw) + this->baseLinkTwist.lY * cos(this->odometryPose.yaw)) * dt;

	//	Update odometry pose
	this->odometryPose.yaw += deltaYaw;
	this->odometryPose.x += deltaX;
	this->odometryPose.y += deltaY;

	//	Update transform
	this->stampedTransform.transform.rotation = tf::createQuaternionMsgFromYaw(this->odometryPose.yaw);
	this->stampedTransform.transform.translation.x = this->odometryPose.x;
	this->stampedTransform.transform.translation.y = this->odometryPose.y;

	//	Update odometry
	this->odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->odometryPose.yaw);
	this->odometry.pose.pose.position.x = this->odometryPose.x;
	this->odometry.pose.pose.position.y = this->odometryPose.y;

	this->odometry.twist.twist.angular.z = deltaYaw;
	this->odometry.twist.twist.linear.x = deltaX;
	this->odometry.twist.twist.linear.y = deltaY;
}

void OdometrySimulator::publish(void)
{
		//	Broadcast transform
	this->stampedTransform.header.stamp = ros::Time::now();
	this->transformBroadcaster.sendTransform(this->stampedTransform);

	//	Publish topic
	this->odometry.header.stamp = ros::Time::now();
	this->odometryPublisher.publish(this->odometry);
}

void OdometrySimulator::makeItSpin(void)
{
	ros::Rate r(10);

	while (ros::ok())
	{
		ros::spinOnce();

		this->update(r.cycleTime());
		this->publish();

		r.sleep();
	}
}
