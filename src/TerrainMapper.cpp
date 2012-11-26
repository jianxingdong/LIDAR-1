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

	//	Setup cell map
	//	***********************************************************
	//	x-y dimension 0.5m * 1.0m @ 0.02m resolution
	//			  	  (x)  *  (y)
	//	z-resolution: 5mm, 0-255, 128 = 0, -0.64m - 0.64m
	//	***********************************************************
	this->depth 			= 0.5;		//	[m]
	this->width 			= 1.0;		//	[m]
	this->cellResolution 	= 0.02;		//	[m]

	this->referenceColor	= 128;
	this->zResolution		= 0.005;	//	[m]

	this->maxX = (int)(depth / cellResolution);
	this->maxY = (int)(width / cellResolution);

	this->image = cv::Mat::zeros(maxX, maxY, CV_8UC1);

	this->resetMap();
	//	Cell setup done

	this->output.encoding = sensor_msgs::image_encodings::MONO8;
}

TerrainMapper::~TerrainMapper()
{

}

void TerrainMapper::makeItSpin (void)
{
	ros::Rate r(10);

	while (ros::ok())
	{
		ros::spinOnce();

		this->output.header.stamp = ros::Time::now();
		this->output.image = this->image;

		this->imagePublisher.publish(this->output.toImageMsg());
	}
}

void TerrainMapper::resetMap (void)
{
	for (int i = 0; i < maxX; i++) for (int j = 0; j < maxY; j++)
		this->image.at<unsigned char>(i, j) = (unsigned char)this->referenceColor;
}
