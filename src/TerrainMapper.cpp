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
	this->heightPublisher = this->imageTransporter.advertise("/LIDAR/terrainMap", 10);
	this->outputHeights.header.frame_id = "terrain_link";
	this->outputHeights.encoding = sensor_msgs::image_encodings::MONO8;

	this->weightPublisher = this->imageTransporter.advertise("/LIDAR/weightMap", 10);
	this->outputWeights.header.frame_id = "terrain_link";
	this->outputWeights.encoding = sensor_msgs::image_encodings::TYPE_64FC1;

	this->videoPublisher = this->imageTransporter.advertise("/LIDAR/videoMap", 10);
	this->videoOutput.header.frame_id = "video_link";
	this->videoOutput.encoding = sensor_msgs::image_encodings::TYPE_64FC1;

	//	Setup callbacks
	this->odometrySubscriber = this->nodeHandler.subscribe("/LIDAR/simulatedOdometry", 10, &TerrainMapper::odometryCallback, this);
	this->imuSubscriber = this->nodeHandler.subscribe("/LIDAR/synched/imu", 10, &TerrainMapper::imuCallback, this);
	this->pointCloudSubscriber = this->nodeHandler.subscribe("/LIDAR/pointCloud", 10, &TerrainMapper::pointCloudCallback, this);

	//	Initializers
	this->initCellMap();
	this->initWeightMap();
	this->initFilter();

	//	Initialize video
	this->video = cv::Mat::zeros(this->heightMap.rows * 2, this->heightMap.cols, cv::DataType<double>::type);

	//	Setup previous time (setup odometry variables)
	this->previousTime = ros::Time::now();
	this->linearDistanceMoved = 0.0;
}

TerrainMapper::~TerrainMapper()
{

}

void TerrainMapper::initCellMap(void)
{
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

	this->minHeight = -(double)(this->refColor) * this->zRes;
	this->maxHeight = (double)(255 - this->refColor) * this->zRes;

	this->heightMap = cv::Mat::zeros(this->imageHeight, this->imageWidth, CV_8UC1);

	this->resetHeights();
	//	Setup cell map done
}

void TerrainMapper::initWeightMap(void)
{
	//	Setup weight window
	this->weightMap = cv::Mat::zeros(this->heightMap.rows, this->heightMap.cols, cv::DataType<double>::type);

	this->resetWeights();
	//	Setup weight window done
}

void TerrainMapper::initFilter(void)
{
	//	Weights of filter window
	this->filterWidth = 0.1;	//	[m]
	this->filterHeight = 0.1;	//	[m]
	this->stdX = 0.01;			//	[m]
	this->stdY = 0.01;			//	[m]
	this->filterWindow = cv::Mat((int)(this->filterHeight / this->xRes), (int)(this->filterWidth / this->yRes), cv::DataType<double>::type);

	double x, y, w;

	for (int i = 0; i < this->filterWindow.rows; i++) for (int j = 0; j < this->filterWindow.cols; j++)
	{
		x = (this->xRes + 2 * this->xRes * i - this->filterHeight) / 2.0;
		y = (this->yRes + 2 * this->yRes * j - this->filterWidth) / 2.0;
		w = Models::Filters::gaussianFilter2D(x, y, this->stdX, this->stdY, 1.0);

		this->filterWindow.at<double>(i, j) = w;
	}
	//	Filter setup done
}

void TerrainMapper::makeItSpin (void)
{
	ros::Rate r(100);

	while (ros::ok())
	{
		ros::spinOnce();

		this->outputHeights.header.stamp = ros::Time::now();
		this->outputHeights.image = this->heightMap;
		this->heightPublisher.publish(this->outputHeights.toImageMsg());

		this->outputWeights.header.stamp = ros::Time::now();
		this->outputWeights.image = this->weightMap;
		this->weightPublisher.publish(this->outputWeights.toImageMsg());

		r.sleep();
	}
}

void TerrainMapper::resetHeights (void)
{
	for (int i = 0; i < this->heightMap.rows; i++) for (int j = 0; j < this->heightMap.cols; j++)
		this->heightMap.at<unsigned char>(i, j) = (unsigned char)this->refColor;
}

void TerrainMapper::resetWeights (void)
{
	for (int i = 0; i < this->weightMap.rows; i++) for (int j = 0; j < this->weightMap.cols; j++)
		this->weightMap.at<double>(i, j) = 1.0;
}

void TerrainMapper::pointCloudToMap (void)
{
	//	Timing statistics


	double x, y, z, weight, newWeight, lookingAtColor, oldWeight;
	int currentX, currentY, lookingAtX, lookingAtY;

	for (unsigned int p = 0; p < this->pointCloud.points.size(); p++)
	{
		x = (double)this->pointCloud.points.at(p).x;
		y = (double)this->pointCloud.points.at(p).y;
		z = (double)this->pointCloud.points.at(p).z;

		currentX = (int)(-x / this->xRes);
		currentY = (int)((-y + (this->width / 2)) / this->yRes);

		if (	currentX >= 0 && currentX < this->heightMap.rows &&
				currentY >= 0 && currentY < this->heightMap.cols)
		{
			//	Convolution with 2D gaussian
			for (int i = 0; i < this->filterWindow.rows; i++)
			{
				for (int j = 0; j < this->filterWindow.cols; j++)
				{
					//	Get weight and calculate position
					weight = this->filterWindow.at<double>(i, j);
					lookingAtX = currentX - (this->filterWindow.rows / 2) + i;
					lookingAtY = currentY - (this->filterWindow.cols / 2) + j;

					//	Checking if x-y coordinate is in bound
					if (	lookingAtX >= 0 && lookingAtX < this->heightMap.rows &&
							lookingAtY >= 0 && lookingAtY < this->heightMap.cols)
					{
						//	Get weight from weight map
						oldWeight = this->weightMap.at<double>(lookingAtX, lookingAtY);

						//	Set pixel color
						lookingAtColor = (double)(this->heightMap.at<unsigned char>(lookingAtX, lookingAtY) - this->refColor) * this->zRes;
						lookingAtColor = (lookingAtColor + z * weight) / (1.0 + weight); //	Updated color

						//	Check for saturation on weight
						newWeight = oldWeight + weight;
						if (newWeight > 8) newWeight = 8;

						//	Check for possibility in overflow when converting to color (unsigned char)
						lookingAtColor < this->minHeight ? lookingAtColor = this->minHeight : lookingAtColor;
						lookingAtColor > this->maxHeight ? lookingAtColor = this->maxHeight : lookingAtColor;

						//	Set color in height map and weight in weight map
						this->heightMap.at<unsigned char>(lookingAtX, lookingAtY) = (unsigned char)(lookingAtColor / this->zRes) + this->refColor;
						this->weightMap.at<double>(lookingAtX, lookingAtY) = newWeight;
					}
				}
			}
		}
	}
}

void TerrainMapper::odometryToMap (void)
{
	this->linearDistanceMoved += this->odometry.twist.twist.linear.x * (ros::Time::now() - this->previousTime).toSec();
	this->angularRotationMoved += this->odometry.twist.twist.angular.z * (ros::Time::now() - this->previousTime).toSec();

	if (this->linearDistanceMoved > this->xRes)
	{
		int pixels = int(this->linearDistanceMoved / this->xRes) + 1;
		this->linearDistanceMoved = 0.0;

		//	Move pixels according to linear translation
		if (this->odometry.twist.twist.linear.x > 0.0)	// Positive
		{
			for (int i = this->heightMap.rows - 2; i >= 0; i--)
			{
				for (int j = 0; j < this->heightMap.cols; j++)
				{
					this->heightMap.at<unsigned char>(i + 1, j) = this->heightMap.at<unsigned char>(i, j);
					this->weightMap.at<double>(i + 1, j) = this->weightMap.at<double>(i, j);
				}
			}
		}
	}

	double dtAng = this->odometry.twist.twist.angular.z * (ros::Time::now() - this->previousTime).toSec();
	if (fabs(dtAng) > 0.0)
	{
		///this->angularRotationMoved = 0.0;

		cv::Mat transform = cv::getRotationMatrix2D(cv::Point2f(this->heightMap.cols / 2, 250.0), dtAng * .1, 1.0);
		cv::Mat dst = cv::Mat::zeros(this->imageHeight, this->imageWidth, CV_8UC1);
		cv::warpAffine(this->heightMap, dst, transform, this->heightMap.size());
		this->heightMap = dst;

		dst = cv::Mat::zeros(this->heightMap.rows, this->heightMap.cols, cv::DataType<double>::type);
		cv::warpAffine(this->weightMap, dst, transform, this->weightMap.size());
		this->weightMap = dst;
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
