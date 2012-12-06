/*
 * TerrainMapper.cpp
 *
 *  Created on: 24 Nov 2012
 *      Author: kent
 */

#include "TerrainMapper.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "TerrainMapper");

	TerrainMapper tMappper;
	tMappper.makeItSpin();

	return 0;
}

TerrainMapper::TerrainMapper() : imageTransporter(this->nodeHandler)
{
	this->imagePublisher = this->imageTransporter.advertise("/LIDAR/terrainMap", 10);
	this->output.header.frame_id = "terrain_link";
	this->output.encoding = sensor_msgs::image_encodings::MONO8;

	//	Setup callbacks
	this->odometrySubscriber = this->nodeHandler.subscribe("/LIDAR/simulatedOdometry", 10, &TerrainMapper::odometryCallback, this);
	this->imuSubscriber = this->nodeHandler.subscribe("/LIDAR/synched/imu", 10, &TerrainMapper::imuCallback, this);
	this->pointCloudSubscriber = this->nodeHandler.subscribe("/LIDAR/pointCloud", 10, &TerrainMapper::pointCloudCallback, this);

	//	Setup cell map
	//	***********************************************************
	//	x:	0.5m @ 0.02m	y:	1.0m @ 0.02m
	//	z:	-0.64m to 0.64m @ 0.005m
	//	***********************************************************
	this->depth 	= 2.0;		//	[m]
	this->xRes		= 0.01;		//	[m]
	this->width 	= 2.0;		//	[m]
	this->yRes		= 0.01;		//	[m]

	this->refColor	= 128;
	this->zRes		= 0.002;	//	[m]

	this->imageHeight	= (int)(depth / xRes);
	this->imageWidth 	= (int)(width / yRes);

	this->minHeight = -(double)(this->refColor) / this->zRes;
	this->maxHeight = (double)(255 - this->refColor) / this->zRes;

	this->image = cv::Mat::zeros(this->imageHeight, this->imageWidth, CV_8UC1);

	this->resetMap(&this->image);
	//	Setup cell map done

	//	Weights of filter window
	this->filterWidth = 0.1;	//	[m]
	this->filterHeight = 0.1;	//	[m]
	this->filterWindow = cv::Mat((int)(this->filterHeight / this->xRes), (int)(this->filterWidth / this->yRes), cv::DataType<double>::type);

	double x, y, w;

	for (int i = 0; i < this->filterWindow.rows; i++) for (int j = 0; j < this->filterWindow.cols; j++)
	{
		x = (this->xRes + 2 * this->xRes * i - this->filterHeight) / 2.0;
		y = (this->yRes + 2 * this->yRes * j - this->filterWidth) / 2.0;
		w = Models::Filters::gaussianFilter2D(x, y, 0.01, 0.01, 1.0);	//	stdX = 0.01m and stdY = 0.01m

		this->filterWindow.at<double>(i, j) = w;
	}
	//	Filter setup done

	//	Setup previous time (setup odometry variables)
	this->previousTime = ros::Time::now();
	this->linearDistanceMoved = 0.0;

}

TerrainMapper::~TerrainMapper()
{

}

void TerrainMapper::makeItSpin (void)
{
	ros::Rate r(100);

	while (ros::ok())
	{
		ros::spinOnce();

		this->output.header.stamp = ros::Time::now();
		this->output.image = this->image;

		this->imagePublisher.publish(this->output.toImageMsg());

		r.sleep();
	}
}

void TerrainMapper::resetMap (cv::Mat *img)
{
	for (int i = 0; i < img->rows; i++) for (int j = 0; j < img->cols; j++)
		this->image.at<unsigned char>(i, j) = (unsigned char)this->refColor;
}

unsigned char TerrainMapper::getPixel (int x, int y, cv::Mat *img)
{
	if (x >= 0 && x < img->rows &&	y >= 0 && y < img->cols)
		return this->image.at<unsigned char>(x, y);

	return 0;
}

void TerrainMapper::setPixel (int x, int y, unsigned char color, cv::Mat *img)
{
	if (x >= 0 && x < img->rows &&	y >= 0 && y < img->cols)
		this->image.at<unsigned char>(x, y) = color;
}

void TerrainMapper::pointCloudToMap (void)
{
	double x, y, z, weight, lookingAtColor;
	int currentX, currentY, lookingAtX, lookingAtY;

	for (unsigned int p = 0; p < this->pointCloud.points.size(); p++)
	{
		x = (double)this->pointCloud.points.at(p).x;
		y = (double)this->pointCloud.points.at(p).y;
		z = (double)this->pointCloud.points.at(p).z;

		currentX = (int)(-x / this->xRes);
		currentY = (int)((-y + (this->width / 2)) / this->yRes);

		if (	currentX >= 0 && currentX < this->image.rows &&
				currentY >= 0 && currentY < this->image.cols)
		{
			//	Convolution with 2D gaussian
			for (int i = 0; i < this->filterWindow.rows; i++)
			{
				for (int j = 0; j < this->filterWindow.cols; j++)
				{
					weight = this->filterWindow.at<double>(i, j);

					lookingAtX = currentX - (this->filterWindow.rows / 2) + i;
					lookingAtY = currentY - (this->filterWindow.cols / 2) + j;

					lookingAtColor = ((double)this->getPixel(lookingAtX, lookingAtY, &this->image) - (double)this->refColor) * this->zRes;

					lookingAtColor = (lookingAtColor + z * weight) / (1.0 + weight); //	Updated color

					//	Check for possibility in oveflow when converting to color (unsigned char)
					lookingAtColor < this->minHeight ? lookingAtColor = this->minHeight : lookingAtColor;
					lookingAtColor > this->maxHeight ? lookingAtColor = this->maxHeight : lookingAtColor;

					this->setPixel(lookingAtX, lookingAtY, (unsigned char)(lookingAtColor / this->zRes) + this->refColor, &this->image);
				}
			}
		}
	}
}

void TerrainMapper::odometryToMap (void)
{

	this->linearDistanceMoved += this->odometry.twist.twist.linear.x * (ros::Time::now() - this->previousTime).toSec();

	if (this->linearDistanceMoved > this->xRes)
	{
		this->linearDistanceMoved = 0.0;

		//	Move pixels according to linear translation
		if (this->odometry.twist.twist.linear.x > 0.0)	// Positive
		{
			for (int i = this->image.rows - 2; i >= 0; i--)
			{
				for (int j = 0; j < this->image.cols; j++)
				{
					this->image.at<unsigned char>(i + 1, j) = this->image.at<unsigned char>(i, j);
				}
			}
		}
	}
}

void TerrainMapper::odometryCallback(const nav_msgs::Odometry::ConstPtr& data)
{
	this->odometry.header.frame_id = data.get()->header.frame_id;
	this->odometry.header.stamp = data.get()->header.stamp;
	this->odometry.pose = data.get()->pose;
	this->odometry.twist = data.get()->twist;

	this->odometryToMap();
}

void TerrainMapper::imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
	this->imu.header.frame_id = data.get()->header.frame_id;
	this->imu.orientation = data.get()->orientation;
	this->imu.orientation_covariance = data.get()->orientation_covariance;
}

void TerrainMapper::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& data)
{
	this->pointCloud.header.frame_id = data.get()->header.frame_id;
	this->pointCloud.channels = data.get()->channels;
	this->pointCloud.points = data.get()->points;

	this->pointCloudToMap();
}
