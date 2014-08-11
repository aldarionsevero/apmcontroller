#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <cmath>     /* abs */

#define X_BOUNDARY			2	//ram 0.4 smartv2
#define Y_BOUNDARY			2	//ram 1.1 smartv2
#define Z_BOUNDARY			2	//

/*
 * StateEstimation class (constructor)
 *
 * This constructor does all the initializations for the StateEstimation node. The private part for things that only this class will have access.
 *
 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
 * @date    		JLY 2014
 */
class setpoint_gains_pub {
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber setpoint_subscriber_;
	ros::Publisher setpoint_publisher_;
	ros::Subscriber gains_subscriber_;
	ros::Publisher gains_publisher_;

	geometry_msgs::Pose new_setpoint_;

	struct {
		double x;
		double y;
		double z;
		double yaw;
	} Setpoint;

public:
	setpoint_gains_pub() {
		ros::NodeHandle params("~");

		setpoint_subscriber_ = node_handle_.subscribe("setpoint", 1,
				&setpoint_gains_pub::setpointCallback, this);
		setpoint_publisher_ = node_handle_.advertise<geometry_msgs::Pose>(
				"setpoint", 1);
//		gains_subscriber_ = node_handle_.subscribe("gains", 1,
//				&setpoint_gains_pub::gainsCallback, this);
//		gains_publisher_ = node_handle_.advertise</*geometry_msgs::Pose*/>(
//				"/gains", 1);
	}

	~setpoint_gains_pub() {
	}

	/*
	 * setpointCallback
	 *
	 * Function called when the topic /Apm/unfiltered_pose (actually the
	 * name of the topic that you set under mocap_optitrack config file)
	 * changes. This function gives an estimation of the current state of
	 * the vehicle.
	 *
	 * @param			<geometry_msgs::Pose> pose from topic /Apm/unfiltered_pose.
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	void setpointCallback(const geometry_msgs::Pose setpoint) {
		// Coordinate frame: absolute world frame.
		// Since a pose is used as input, we have to deal with the quaternion.
		tf::Quaternion q(setpoint.orientation.x, setpoint.orientation.y,
				setpoint.orientation.z, setpoint.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		Setpoint.yaw = yaw;

		new_setpoint_ = setpoint;

		ROS_INFO("Current Setpoint is x y z: %f %f %f", setpoint.position.x, setpoint.position.y,
				setpoint.position.z);
		ROS_INFO("Please enter new Set point for x: ");
		scanf("%lf", &new_setpoint_.position.x);
		ROS_INFO("Please enter new Set point for y: ");
		scanf("%lf", &new_setpoint_.position.y);
		ROS_INFO("Please enter new Set point for z: ");
		scanf("%lf", &new_setpoint_.position.z);

		if (new_setpoint_.position.x > X_BOUNDARY) {
			new_setpoint_.position.x = X_BOUNDARY;
		}
		if (new_setpoint_.position.x < -X_BOUNDARY) {
			new_setpoint_.position.x = -X_BOUNDARY;
		}
		if (new_setpoint_.position.y > Y_BOUNDARY) {
			new_setpoint_.position.y = Y_BOUNDARY;
		}
		if (new_setpoint_.position.y< -Y_BOUNDARY) {
			new_setpoint_.position.y = -Y_BOUNDARY;
		}

		setpointPublish();

	}
	void setpointPublish() {
		setpoint_publisher_.publish(new_setpoint_);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "setpoint_gains_pub");
	setpoint_gains_pub setpoint_gains_pub;
	ros::spin();

	return 0;
}
