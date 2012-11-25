/*
 * KeyboardEventListener.cpp
 *
 *  Created on: Nov 13, 2012
 *      Author: kent
 */

#include "KeyboardEventListener.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "KeyboardEventListener");

	KeyboardEventListener keyboardEventListener;
	keyboardEventListener.listen();

	ros::spin();

	return 0;
}

KeyboardEventListener::KeyboardEventListener()
{
	//	Setup signal
	signal(SIGINT, KeyboardEventListener::quit);

	this->nodeHandler = ros::NodeHandle();
	this->keyPublisher = this->nodeHandler.advertise<std_msgs::Char>("/fmHMI/keyboardEventListener", 100);

	this->kfd = 0;
	this->key = 0;

	//	Get raw terminal attributes
	tcgetattr(this->kfd, &this->raw);
	memcpy(&this->cooked, &this->raw, sizeof(struct termios));

	//	Cook up new terminal attributes
	this->cooked.c_lflag &= ~(ICANON | ECHO);
	this->cooked.c_cc[VEOL] = 1;
	this->cooked.c_cc[VEOF] = 2;

	//	Set cooked terminal attributes
	tcsetattr(this->kfd, TCSANOW, &this->cooked);
}

KeyboardEventListener::~KeyboardEventListener()
{

}

void KeyboardEventListener::listenForEvents(void)
{
	printf(" Listening for keyboard events ... \n");

	isListening = true;

	do
	{
		// Get events from keyboard
		if (read(this->kfd, &this->key, 1) < 0)
		{
			printf(" Error occured reading keyboard, shutting down node ... \n");
			ros::shutdown();
			exit(-1);
		}

		//	Publish keyboardEvent
		ROS_DEBUG("%c", this->key);
		this->msg.data = this->key;
		keyPublisher.publish(this->msg);
	} while (ros::ok() && isListening);

	printf(" Killing node due to signal: %i ... \n", signalId);

	//	Set cooked terminal attributes
	tcsetattr(this->kfd, TCSANOW, &this->raw);

	//	Shutdown node
	ros::shutdown();
	exit(signalId);
}

void KeyboardEventListener::quit(int signal)
{
	isListening = false;
	signalId = signal;
}
