#include <ros/ros.h>
#include "roscopter/RC.h"

roscopter::RC buildRCMsg(int aileron, int elevator, int throttle, int yaw) {
	roscopter::RC msg;
	msg.channel[0] = aileron;
	msg.channel[1] = elevator;
	msg.channel[2] = throttle;
	msg.channel[3] = yaw;

	//We never override channel 4, this will let us recover to manual control.
	msg.channel[4] = 0;

	//Channels 5-7 are not in use, so we set them = 0.
	msg.channel[5] = 0;
	msg.channel[6] = 0;
	msg.channel[7] = 0;

	return msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_sender");
    ros::NodeHandle n;
    ros::Publisher rc_pub = n.advertise<roscopter::RC>("send_rc", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
	roscopter::RC msg;
	//roll/aileron, pitch/elevator, throttle, yaw/rudder
	msg = buildRCMsg(0,0,1500,0);
	rc_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	++count;
    }
    return 0;
}
