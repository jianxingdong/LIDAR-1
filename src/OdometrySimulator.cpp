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
	odomSim.makeItSpin();

	return 0;
}

OdometrySimulator::OdometrySimulator()
{
	//	ROS node handler stuff
	this->nodeHandler = ros::NodeHandle();
	this->odometryPublisher = this->nodeHandler.advertise<nav_msgs::Odometry>("/LIDAR/simulatedOdometry", 10);
	this->twistSubscriber = this->nodeHandler.subscribe("/fmHMI/keyboardToTwist", 10, &OdometrySimulator::twistCallback, this);

	//	Initialize transform
	this->stampedTransform.header.frame_id = "/odom";
	this->stampedTransform.child_frame_id = "/base_link";
	this->stampedTransform.header.stamp = ros::Time::now();

	this->stampedTransform.transform.rotation = tf::createQuaternionMsgFromYaw(0.0f);
	this->stampedTransform.transform.translation.x = 0.0f;
	this->stampedTransform.transform.translation.y = 0.0f;
	this->stampedTransform.transform.translation.z = 0.0f;

	//	Initialize odometry
	this->odometry.header.frame_id = "/odom";
	this->odometry.child_frame_id = "/base_link";
	this->odometry.header.stamp = ros::Time::now();

	this->odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0f);
	this->odometry.pose.pose.position.x =
	this->odometry.pose.pose.position.y =
	this->odometry.pose.pose.position.z = 0.0f;
}

OdometrySimulator::~OdometrySimulator()
{

}

void OdometrySimulator::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& data)
{
	this->twist.header = data.get()->header;
	this->twist.twist = data.get()->twist;

	this->update((this->twist.header.stamp - this->oldTime).toSec());

	this->oldTime = this->twist.header.stamp;
}

void OdometrySimulator::update(double dt)
{
	//	Calculate odometry
	double deltaYaw = this->twist.twist.angular.z * dt;
	double deltaX = (this->twist.twist.linear.x * cos(this->twist.twist.angular.z) - this->twist.twist.linear.x * sin(this->twist.twist.angular.z)) * dt;
	double deltaY = (this->twist.twist.linear.x * sin(this->twist.twist.angular.z) + this->twist.twist.linear.x * cos(this->twist.twist.angular.z)) * dt;

	//	Update transform
	this->stampedTransform.transform.rotation =	tf::createQuaternionMsgFromYaw(tf::getYaw(this->stampedTransform.transform.rotation) + deltaYaw);
	this->stampedTransform.transform.translation.x += deltaX;
	this->stampedTransform.transform.translation.y += deltaY;

	//	Update odometry
	this->odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(this->odometry.pose.pose.orientation) + deltaYaw);
	this->odometry.pose.pose.position.x += deltaX;
	this->odometry.pose.pose.position.y += deltaY;

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

		this->publish();

		r.sleep();
	}
}
