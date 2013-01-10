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
	this->nodeHandler = ros::NodeHandle("~");

	this->nodeHandler.param<std::string>("twist_topic", 	this->twistTopic, 	"/fmHMI/keyboardToTwist");
	this->nodeHandler.param<std::string>("odom_topic", 	 	this->odomTopic, 	"/LIDAR/simulatedOdometry");

	this->odometryPublisher = this->nodeHandler.advertise<nav_msgs::Odometry>(this->odomTopic, 10);
	this->twistSubscriber = this->nodeHandler.subscribe(this->twistTopic, 10, &OdometrySimulator::twistCallback, this);

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

	this->x =
	this->y =
	this->theta = 0.0f;
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
	double deltaX = (this->twist.twist.linear.x * cos(this->theta) - this->twist.twist.linear.y * sin(this->theta)) * dt;
	double deltaY = (this->twist.twist.linear.x * sin(this->theta) + this->twist.twist.linear.y * cos(this->theta)) * dt;

	this->x += deltaX;
	this->y += deltaY;
	this->theta += deltaYaw;

	//	Update transform
	this->stampedTransform.transform.translation.x = this->x;
	this->stampedTransform.transform.translation.y = this->y;
	this->stampedTransform.transform.translation.z = 0.0f;
	this->stampedTransform.transform.rotation =	tf::createQuaternionMsgFromYaw(this->theta);

	//	Update odometry
	this->odometry.pose.pose.position.x = this->x;
	this->odometry.pose.pose.position.y = this->y;
	this->odometry.pose.pose.position.z = 0.0f;
	this->odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->theta);

//	this->odometry.twist.twist.linear.x = deltaX;
//	this->odometry.twist.twist.linear.y = deltaY;
//	this->odometry.twist.twist.angular.z = deltaYaw;
	this->odometry.twist.twist.linear.x = this->twist.twist.linear.x;
	this->odometry.twist.twist.linear.y = 0.0;
	this->odometry.twist.twist.angular.z = this->twist.twist.angular.z;
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
