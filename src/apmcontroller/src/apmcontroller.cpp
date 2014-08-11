#include <ros/ros.h>
#include "roscopter/RC.h"
#include "roscopter/Control.h"
//#include "roscopter/APMCommand.h"
#include "UdpBridgeRecv.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

//network defines
#define UDP_IP             "127.0.0.1" //     "192.168.0.103" //       // Target IP
#define UDP_PORT_XMIT       51001                  // Port no.
#define UDP_PORT_RECV       51001                  // Port no.

//RC values defines
#define CH_5_TOGGLE_VALUE	1800
#define RC_ROLL_LOW			1128
#define RC_ROLL_MIDDLE		1527
#define RC_ROLL_HIGH		1926

#define RC_PITCH_LOW		1128
#define RC_PITCH_MIDDLE		1527
#define RC_PITCH_HIGH		1926

#define RC_THROTTLE_LOW		1128
#define RC_THROTTLE_MIDDLE	1527
#define RC_THROTTLE_HIGH	1926

//gravity compensation
#define RC_THROTTLE_OFFSET	1470

#define RC_YAW_LOW			1130
#define RC_YAW_MIDDLE		1529
#define RC_YAW_HIGH			1928

//TODO put all this things in structs

//shutdown and manual control of throttle toggles
static struct {
	bool shut;
	bool yawManual;
	bool throttleManual;
	bool xbox;
	bool arm;
	bool disarm;
} Toggles;

//attitudes to send
static struct {
	int roll_send;
	int pitch_send;
	int throttle_send;
	int yaw_send;
} AttSend;

//attitudes that comes from the priority switch (mux?). selects between joystick and controller (station). Priority on Joystick
static struct {
	double mux_roll;
	double mux_pitch;
	double mux_throttle;
	double mux_yaw;
} AttMux;

//attitudes directly from the controller (not from the switch(mux))
static struct {
	double cont_roll;
	double cont_pitch;
	double cont_throttle;
	double cont_yaw;
} AttCont;

//attitudes directly from the joystick  (not from the switch(mux))
static struct {
	double joy_roll;
	double joy_pitch;
	double joy_throttle;
	double joy_yaw;
} Attjoy;

//what joystick to use
int joyinput;

/*
 * buildRCMsg
 *
 * Builds the <roscopter::RC> message to send.
 *
 * @param aileron   Aileron (roll) value to be sent
 * @param elevator  Elevator (pitch) value to be sent
 * @param throttle 	Throttle value to be sent
 * @param yaw  		Yaw value to be sent .
 * @return          Complete <roscopter::RC> message to be sent
 * @author  		Lucas Severo <lucassalves65@gmail.com>
 * @date    		JLY 2014
 */
roscopter::RC buildRCMsg(int aileron, int elevator, int throttle, int yaw) {

	//roll/aileron, pitch/elevator, throttle, yaw/rudder
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

/*
 * All map functions
 *
 * Each one of them working for a type of variable.
 * Maps input values to a new range.
 * mapInitialThrottledbworking mapping low and high throttle to 0..1
 *
 * @param x   		Value to be mapped
 * @param in_min    Minimal value expected from the input
 * @param in_max 	Maximal value expected from the input
 * @param out_min  	Wanted minimal value
 * @param out_max  	Wanted maximal value
 * @return          Mapped value
 * @author  		Lucas Severo <lucassalves65@gmail.com>
 * @date    		JLY 2014
 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int mapint(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double mapdb(double x, double in_min, double in_max, double out_min,
		double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//1228..1926 -> 0..1
double mapInitialThrottledb(double InitialThrottle, double LowThrottle,
		double HighThrottle) {
	return (((((HighThrottle / LowThrottle) - 1)) * InitialThrottle) + 1)
			* LowThrottle;
}

/*
 * rcCallback
 *
 * Function called every time that the /rc topic changes values.
 * Here we monitor the channel 5, if it changes, we want to shutdown (RC transmitter getting total control).
 *
 * @param msg		<roscopter::RC> message from /rc topic (Roscopter node)
 * @author  		Lucas Severo <lucassalves65@gmail.com>
 * @date    		JLY 2014
 */
void rcCallback(const roscopter::RC::ConstPtr& msg) {
	if (msg->channel[4] < CH_5_TOGGLE_VALUE)
		Toggles.shut = true;
}

/*
 * velCAllback
 *
 * Function called every time that the /output/cmd_vel topic changes.
 * Here we monitor the output of the cmd_vel_mux node, that is a node that
 * monitors the output from the fly_from_joystick (topic /cmd_vel_joy) and
 * controller (topic /cmd_vel_controller) nodes, and sends one of them to
 * /output/cmd_vel.
 *
 * Obs: It gives priority to the joystick.
 *
 * @param msg		<geometry_msgs::Twist> message from /output/cmd_vel topic (cmd_vel_mux node)
 * @author  		Lucas Severo <lucassalves65@gmail.com>
 * @date    		JLY 2014
 */
void velCAllback(const geometry_msgs::TwistConstPtr &msg) {
	AttMux.mux_roll = (msg->linear.x);
	AttMux.mux_pitch = (msg->linear.y);
	AttMux.mux_throttle = (msg->linear.z); //
	AttMux.mux_yaw = (msg->angular.z);

}

/*
 * velContCAllback
 *
 * Function called every time that the /cmd_vel_cont topic changes.
 * Here we monitor the output of the controller node.
 *
 * @param		<geometry_msgs::Twist> message from /cmd_vel_cont topic (controller node)
 * @author  	Lucas Severo <lucassalves65@gmail.com>
 * @date    	JLY 2014
 */
void velContCAllback(const geometry_msgs::TwistConstPtr &msg) {
	AttCont.cont_roll = (msg->linear.x);
	AttCont.cont_pitch = (msg->linear.y);
	AttCont.cont_throttle = (msg->linear.z);
	AttCont.cont_yaw = (msg->angular.z);
}

/*
 * velJoyCAllback
 *
 * Function called every time that the /cmd_vel_joy topic changes.
 * Here we monitor the output of the fly_from_joystick node.
 *
 * @param		<geometry_msgs::Twist> message from /cmd_vel_joy topic (fly_from_joystick node)
 * @author  	Lucas Severo <lucassalves65@gmail.com>
 * @date    	JLY 2014
 */
void velJoyCAllback(const geometry_msgs::TwistConstPtr &msg) {
	Attjoy.joy_roll = (msg->linear.x);
	Attjoy.joy_pitch = (msg->linear.y);
	Attjoy.joy_throttle = (msg->linear.z); //
	Attjoy.joy_yaw = (msg->angular.z);
}

/*
 * joyCallback
 *
 * Function called every time that the /joy topic changes.
 * Here we monitor the output of the joy_node node. Keeps
 * track of all axis and butons of the joystick (Logitec
 * 3d or Xbox360 suported). If button 10 is pressed, we
 * want the system to shutdown, and then the transmiter
 * has control. If buton 11 is pressed, we want to give
 * total control to the ground station (apmcontroller and
 * controller nodes).
 *
 * @param		<sensor_msgs::Joy> message from /joy topic (joy_node node)
 * @author  	Lucas Severo <lucassalves65@gmail.com>
 * @date    	JLY 2014
 */
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	//if button 10 is pressed, we want the system to shutdown
	if (joy->buttons[9] == 1) {
		Toggles.shut = true;
		//ROS_INFO("shut");
	}

	//manual control
	if (joy->buttons[10] == 1) {
		Toggles.throttleManual = false;
	}

//	//if button 7 is pressed, arm
//	if (joy->buttons[2] == 1) {
//		Toggles.arm = true;
//		//ROS_INFO("arm");
//	}
//
//	//button 8 disarm
//	if (joy->buttons[3] == 1) {
//		Toggles.disarm = true;
//	}
}

int main(int argc, char **argv) {

	//commands to send (arm included)
	enum Commands {
		CMD_LAUNCH = 1,
		CMD_LAND = 2,
		CMD_ARM = 3,
		CMD_DISARM = 4,
		CMD_SET_STABILIZE = 5,
		CMD_SET_ALT_HOLD = 6,
		CMD_SET_AUTO = 7,
		CMD_SET_LOITER = 8,
		CMD_SET_LAND = 9,
		RETURN_RC_CONTROL = 10
	};

	//initialization of variables
	Toggles.shut = false;
	Toggles.throttleManual = true;
	Toggles.xbox = true;
	Toggles.arm = false;

	AttSend.roll_send = 0;
	AttSend.pitch_send = 0;
	AttSend.throttle_send = 0;
	AttSend.yaw_send = 0;
	Attjoy.joy_throttle = -1;

	//checking what joystick to use
	ROS_INFO("xbox or lojitec? 1 - xbox; 2 - logitec");
	scanf("%d", &joyinput);
	if (joyinput == 1) {
		//xbox controller
		printf("xbox\n");
		Toggles.xbox = true;
	} else if (joyinput == 2) {
		//Logitec joystick
		printf("logitec\n");
		Toggles.xbox = false;
	}

	ros::init(argc, argv, "rc_sender");
	ROS_INFO("Initializing");

	//declaring the node itself
	ros::NodeHandle n;

	//uncomment this for udp receive (if using simulink)
	//UdpBridgeRecv recv;

	//publishers and subscribers (talkers and listeners) of the topic used by this node.
	//only publishes to /send_rc, and is this that makes Roscopter send the messages to the vehicle
	//all the listeners calls the callback functions
	ros::Publisher rc_pub = n.advertise<roscopter::RC>("send_rc", 1000);
	ros::Subscriber sub = n.subscribe("rc", 1000, rcCallback);
	ros::Subscriber joy_sub_ = n.subscribe("joy", 10, joyCallback);
	ros::Subscriber velocity_listener = n.subscribe("output/cmd_vel", 1,
			velCAllback);
	ros::Subscriber velocity_listenerJoy = n.subscribe("cmd_vel_joy", 1,
			velJoyCAllback);
	ros::Subscriber velocity_listenerController = n.subscribe(
			"cmd_vel_controller", 1, velContCAllback);

	//service to call commands (like arm)
//	ros::ServiceClient apm_cmd_ = n.serviceClient<roscopter::APMCommand>(
//			"command");

	//loop rate for the publishings
	ros::Rate loop_rate(50);

	//counter if number of interactions is needed
	int count = 0;

	ROS_INFO("Entering loop");

	while (ros::ok()) {
		roscopter::RC msg;

		//prepare roll with the right ranges for building the message
		AttSend.roll_send = int(
				mapdb(AttMux.mux_roll, -1, 1, RC_ROLL_LOW, RC_ROLL_HIGH));

		//prepare pitch with the right ranges for building the message
		//inverted: velocity in positive direction for low valuess (arducopter APM specification)
		AttSend.pitch_send = int(
				mapdb(AttMux.mux_pitch, 1, -1, RC_PITCH_LOW, RC_PITCH_HIGH));

		//prepare throttle with the right ranges for building the message
		//if we ARE NOT in the manual control mode, we control the vehicles throttle with the offset + output of controller
		//the joystick only has effect in the AttSend.throttle_send if it is trying to lower the throttle (safety)

		if (Attjoy.joy_throttle > -0.1 && Toggles.throttleManual) {
			Attjoy.joy_throttle = -0.1;
		}
//		if (AttCont.cont_throttle > -0.1 && Toggles.throttleManual) {
//			AttCont.cont_throttle = -0.1;
//		}
		if (!Toggles.throttleManual) {
			if (Attjoy.joy_throttle > 0) {
				AttSend.throttle_send = int(
						mapdb(AttCont.cont_throttle, -1, 1, -450,
								+450)) + RC_THROTTLE_OFFSET;
			} else {
				AttSend.throttle_send =
						int(
								mapdb(AttCont.cont_throttle, -1, 1, -450,
										+450)) + int(mapdb(Attjoy.joy_throttle, -1, 1, -600, 600)) + RC_THROTTLE_OFFSET;
			}

		}

		//if we ARE in the manual control mode, throttle will be totally controlled by the joystick. 0..1 if xbox (because
		//of the spring bringing the stick to the middle). -1..1 if not xbox (so, logitec, with stationary throttle control)
		else {
			//should only be controlled by logitec
			//if(!xbox){
			AttSend.throttle_send = int(
					mapdb(Attjoy.joy_throttle, -1, 1, RC_THROTTLE_LOW,
					RC_THROTTLE_HIGH));
			//}
//		else {
//			AttSend.throttle_send = int(
//					mapInitialThrottledb(Attjoy.joy_throttle,
//							RC_THROTTLE_LOW,
//							RC_THROTTLE_HIGH)); //0..1 instead of -1..1
//		}
		}

		//prepare yaw with the right ranges for building the message
		AttSend.yaw_send = int(
				mapdb(AttCont.cont_yaw, -1, 1, RC_YAW_LOW, RC_YAW_HIGH));

		//build the message and then publish it on the /send_rc (and Roscopter node deals with Mavlink to send to the copter)
		msg = buildRCMsg(AttSend.roll_send, AttSend.pitch_send,
				AttSend.throttle_send, AttSend.yaw_send);
		rc_pub.publish(msg);

		//handle ROS events
		ros::spinOnce();

		//Keep desired frequency
		loop_rate.sleep();

		//if the joyCallback detected the pressing of the buttons[9] (button 10), safely shut down the system
		if (Toggles.shut) {
			//make all attitudes = RC_MIDDLE, and throttle decrementing to RC_THROTTLE_LOW
			msg.channel[0] = RC_ROLL_MIDDLE;
			msg.channel[1] = RC_PITCH_MIDDLE;
			msg.channel[3] = RC_YAW_MIDDLE;
			while ((msg.channel[2] > RC_THROTTLE_LOW)) {
				msg.channel[2] = msg.channel[2] - 10;
				rc_pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}

			//give back controll to RC transmiter (just publish 0 on the channels)
			for (int i = 0; i < 4; ++i) {
				msg.channel[i] = 0;
			}
			msg.channel[3] =RC_YAW_LOW;
			//publish this message to /send_rc (and Roscopter node deals with Mavlink to send to the copter)
			rc_pub.publish(msg);

			//shut down this node
			ros::shutdown();
		}
//		if (Toggles.arm) {
//			roscopter::APMCommand srv;
//			srv.request.command = CMD_ARM;
//			ROS_INFO("Trying to arm");
//			if (apm_cmd_.call(srv))
//				ROS_WARN("Arm request complete");
//			else
//				ROS_ERROR("Arm request incomplete");
//			Toggles.arm=false;
//		}
//		if (Toggles.disarm) {
//			roscopter::APMCommand srv;
//			srv.request.command = CMD_DISARM;
//			ROS_INFO("Trying to arm");
//			if (apm_cmd_.call(srv))
//				ROS_WARN("Arm request complete");
//			else
//				ROS_ERROR("Arm request incomplete");
//			Toggles.disarm=false;
//		}
		++count;
	}

	return 0;
}
