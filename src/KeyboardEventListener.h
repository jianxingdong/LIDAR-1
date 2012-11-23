/*
 * KeyboardEventListener.h
 *
 *  Created on: Nov 13, 2012
 *      Author: kent
 */

#ifndef KEYBOARDEVENTLISTENER_H_
#define KEYBOARDEVENTLISTENER_H_

#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>

bool isListening = false;
int signalId = 0;

class KeyboardEventListener
{
private:
	int kfd;
	char key;
	struct termios cooked, raw;

	ros::NodeHandle nodeHandler;
	ros::Publisher keyPublisher;
	std_msgs::Char msg;

	void listenForEvents(void);
	static void quit(int signal);

public:
	KeyboardEventListener();
	virtual ~KeyboardEventListener();

	void listen(void) { this->listenForEvents(); }
};

#endif /* KEYBOARDEVENTLISTENER_H_ */
