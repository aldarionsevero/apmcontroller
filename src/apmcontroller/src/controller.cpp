/*! \file controller.cpp
 * \brief controller executable for the RAM package
 *
 * This executable implements a PD controller. Gains, non-linear actions, velocity-damping and integrative actions can be set through a Python UI.
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Empty.h>
#include <math.h> 
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
//#include <ram/nonlinearity.h>
#include <cmath>

#define GAIN_MUL			1
#define G_CONSTANT			1
#define X_BOUNDARY			2	//ram 0.4 smartv2
#define Y_BOUNDARY			2	//ram 1.1 smartv2
#define Z_BOUNDARY			2	//

/*
 * Control class (constructor)
 *
 * This constructor does all the initializations for the controller node. The private part for things that only this class will have access.
 *
 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
 * @date    		JLY 2014
 */
class Control {
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber sim_subscriber_;
	ros::Subscriber setpoint_subscriber_;
	ros::Subscriber odom_subscriber_;
	ros::Publisher velocity_publisher_;

	/* ADDITIONAL PUBLISHER FOR GRAPHING PURPOSES */
	ros::Publisher p_publisher_;
	ros::Publisher d_publisher_;
	ros::Publisher x_publisher_;
	ros::Publisher yaw_publisher_;
	ros::Publisher errorx_publisher_;
	ros::Publisher errordx_publisher_;
	ros::Publisher foaw_publisher_;
	ros::Publisher velocity_damping_publisher_;
	/* END ADDITIONAL PUBLISHERS */

	/* Memory */
	std::vector<double> errorsx_; // Previous errors in x
	std::vector<double> errorsy_; // Previous errors in y
	std::vector<double> errorsz_; // Previous errors in z
	std::vector<double> errorsyaw_; // Previous errors in z
	std::vector<double> zprevious_; // Previous z's
	std::vector<double> times_; // Corresponding times
	std::vector<geometry_msgs::Twist> speeds_; // speeds
	geometry_msgs::Twist speed_;
	std::vector<double> yaws_;

	/* Filter parameters */
	// FOAW
	double uncertainty_band_; // Max band around pose to find line fitting
	int error_memory_; // Max errors stored

	// LOW PASS
	double T_, dt_;

	// STATE ESTIMATION
	double K_;

	// Messages
	geometry_msgs::Twist velocity_;
	std_msgs::Empty empty_;

	// Control parameters
	bool hovermode_;
	bool velocity_damping_;
	bool i_action_;
	double hover_treshold_;

	struct {
		double p_z;
		double d_z;
		double p_translational;
		double d_translational;
		double p_rotational;
		double d_rotational;
		double velocity;
		double i;
	} Gains;

	struct {
		double x;
		double y;
		double z;
		double yaw;
	} Setpoint; // Check launch file for additional description of setpoint

	struct {
		double x_translational;
		double y_translational;
		double z_translational;
		double z_rotational;
	} Error;

	struct {
		double x_translational;
		double y_translational;
		double z_translational;
		double z_rotational;
	} Pose;

	struct {
		double x_translational;
		double y_translational;
		double z_translational;
		double z_rotational;
	} ErrorDot;

	// Integral action
	struct {
		double x;
		double y;
	} isum;

	// general variables
	bool simulation;
	double previous_publish_time_;
	int freq_;
	double yaw_; // Screw quaternions

	/*
	 * Control class (constructor)
	 *
	 * This constructor does all the initializations for the controller node. The public part for things that are accessible to all classes.
	 *
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
public:
	Control() {
		ROS_INFO("Initializing controller");
		ros::NodeHandle params("~");

		// If simulation is true, get data feedback from simulation and run simulationCallback. Otherwise, use OptiTrack
		simulation = false;
		params.getParam("simulation", simulation);
		if (simulation) {
			sim_subscriber_ = node_handle_.subscribe("gazebo/model_states", 1,
					&Control::simulationCallback, this);
			ROS_INFO("SIMULATION MODE");
		} else {
			odom_subscriber_ = node_handle_.subscribe("filtered_state", 1,
					&Control::odomCallback, this);
			ROS_INFO("CONTROL MODE");
		}

		// Velocity publisher
		velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>(
				"cmd_vel_controller", 1);

//    /* Non linear part */
//    nonlin_subscriber_ = node_handle_.subscribe("nonlinearity", 1, &Control::nonlinCallback, this);
//    nonlinx = 0;
//    nonliny = 0;
//    nonlinz = 0;

		/* ADDITIONAL PUBLISHER FOR GRAPHING PURPOSES */
		d_publisher_ = node_handle_.advertise<std_msgs::Float32>("control_d",
				1);
		p_publisher_ = node_handle_.advertise<std_msgs::Float32>("control_p",
				1);
		x_publisher_ = node_handle_.advertise<std_msgs::Float32>("tot_x", 1);
		errorx_publisher_ = node_handle_.advertise<std_msgs::Float32>("error_x",
				1);
		errordx_publisher_ = node_handle_.advertise<std_msgs::Float32>(
				"errord_x", 1);
		yaw_publisher_ = node_handle_.advertise<std_msgs::Float32>("yaw", 1);
		foaw_publisher_ = node_handle_.advertise<std_msgs::Int32>("foaw_d", 1);
		velocity_damping_publisher_ = node_handle_.advertise<std_msgs::Float32>(
				"velocity_damping", 1);
		/* END ADDITIOANL PUBLISHERS */

		// Subscriber for setpoint changes
		setpoint_subscriber_ = node_handle_.subscribe("setpoint", 1,
				&Control::setpointCallback, this);

		// Initial values
		previous_publish_time_ = 0;
		freq_ = 50;
		params.getParam("publish_rate", freq_);

		// Hover mode
		hovermode_ = false;
		hover_treshold_ = 0.1;
		params.getParam("hovermode", hovermode_);
		params.getParam("hover_treshold", hover_treshold_);

		// Velocity damping
		velocity_damping_ = false;
		params.getParam("velocity_damping", velocity_damping_);

		/* Filtering */
		// FOAW
		error_memory_ = 15;
		uncertainty_band_ = 0.01;
		params.getParam("error_memory", error_memory_);
		params.getParam("uncertainty_band", uncertainty_band_);

		// STATE ESTIMATION
		K_ = 70;
		params.getParam("K", K_);

		// LOWPASS
		T_ = 0.1; //sec
		dt_ = 0.03; //ms

		// Initial values for setpoint
		Setpoint.x = -0.2;
		params.getParam("setpoint_x", Setpoint.x);
		Setpoint.y = 0.5;
		params.getParam("setpoint_y", Setpoint.y);
		Setpoint.z = 1;
		params.getParam("setpoint_z", Setpoint.z);
		Setpoint.yaw = 0;
		params.getParam("setpoint_yaw", Setpoint.yaw);
		// Controller gains
		Gains.p_z = 0.05 * GAIN_MUL; // it was 0.5
		params.getParam("gain_p_z", Gains.p_z);
		Gains.d_z = 0.01 * GAIN_MUL;
		params.getParam("gain_d_z", Gains.d_z);
		Gains.p_translational = 0.13 * GAIN_MUL; //0.09
		params.getParam("gain_p_translational", Gains.p_translational);
		Gains.d_translational = 0.1 * GAIN_MUL; //it was 0.16, then 0.9
		params.getParam("gain_d_translational", Gains.d_translational);
		Gains.p_rotational = 0.4 * GAIN_MUL;
		params.getParam("gain_p_rotational", Gains.p_rotational);
		Gains.d_rotational = 0.2 * GAIN_MUL;
		params.getParam("gain_d_rotational", Gains.d_rotational);
		Gains.velocity = -0.3 * GAIN_MUL;
		params.getParam("gain_velocity", Gains.velocity);

		// I action
		i_action_ = false; //start using offset
		Gains.i = 0.01 * GAIN_MUL;
		params.getParam("gain_i", Gains.i);
		params.getParam("i_action", i_action_);
		isum.x = 0;
		isum.y = 0;
		ROS_INFO("%f", isum.x);
//		while (ros::ok()) {
//			/////////////////////////////////////////////////////
//			velocity_.linear.x = -1;
//			velocity_.linear.y = -1;
//			velocity_.linear.z = 0.2;
//
//			velocity_.angular.x = -0.5;
//			velocity_.angular.y = -0.5;
//			velocity_.angular.z = -0.5;
//			//////////////////////////////////////////////////////
//
//			velocity_publisher_.publish(velocity_);
//
//		}
	}

	/*
	 * Control class (desctructor)
	 *
	 * Publish empty msg as last message when closed.
	 *
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	~Control() {
		// Publish empty msg as last message when closed
		velocity_ = geometry_msgs::Twist();

		velocity_publisher_.publish(velocity_);
	}

	/*void dynConfCb(controller::ParamsConfig &config, uint32_t level)
	 {
	 Gains.d_translational = config.d_translational;
	 Gains.d_rotational = config.d_rotational;
	 Gains.p_translational = config.p_translational;
	 Gains.p_rotational = config.p_rotational;
	 Gains.p_z = config.p_z;
	 Gains.d_z = config.d_z;
	 }*/

	/*
	 * setpointCallback
	 *
	 * Function called when the topic /setpoint changes. This function sets a new setpoint for the drone to go.
	 *
	 * @param			<geometry_msgs::Pose> setpoint from topic /setpoint. You can use Cees interface to publish to this topic, or use pub in console.
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

		// Check if the setpoint changed. If so, reset I action. This is the only anti wind-up reset.
		if (Setpoint.x == setpoint.position.x
				&& Setpoint.y == setpoint.position.y) {
			isum.x = 0;
			isum.y = 0;
		}
		if (setpoint.position.x > X_BOUNDARY) {
			Setpoint.x = X_BOUNDARY;
		}
		if (setpoint.position.x < -X_BOUNDARY) {
			Setpoint.x = -X_BOUNDARY;
		}
		if (setpoint.position.y > Y_BOUNDARY) {
			Setpoint.y = Y_BOUNDARY;
		}
		if (setpoint.position.y < -Y_BOUNDARY) {
			Setpoint.y = -Y_BOUNDARY;
		} else {
			Setpoint.x = setpoint.position.x;
			Setpoint.y = setpoint.position.y;
			Setpoint.z = setpoint.position.z;
			Setpoint.yaw = yaw;
		}
	}

	/*
	 * odomCallback
	 *
	 * Function called when the topic /filtered_state changes.
	 * This function does all the error calculations regarding
	 * the position of the vehicle and the desired position.
	 * The same for the vehicle velocity and desired velocity.
	 * This topic /filtered_state comes from the state_estimation node.
	 *
	 * @param			<nav_msgs::Odometry> odom from topic /filtered_state.
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	void odomCallback(const nav_msgs::Odometry odom) {
		// Coordinate frame: absolute world frame.
		// Set time when message comes in (used for derivative)
		double current_time;
		current_time = ros::Time::now().toSec();
		times_.push_back(current_time);

		// Prepare variables for error filtering
		double ex, ez, ey, eyaw,znow;

		// Calculate current rotations based on sensor data. This is important, because it is used in all position error calculations. Filter yaw!
		tf::Quaternion q(odom.pose.pose.orientation.x,
				odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
				odom.pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		//yaw_ = lowPassFilter(yaw, yaw_, dt_, T_) ;

		yaws_.push_back(yaw);
		yaw_ = avg(yaws_, 10);
		//yaw_ = yaw;

		std_msgs::Float32 yaw_msg;
		yaw_msg.data = yaw_;
		yaw_publisher_.publish(yaw_msg);

		// Calculate errors. Positive error -> negative action needed
		ex = (Setpoint.x - odom.pose.pose.position.x);
		ey = (Setpoint.y - odom.pose.pose.position.y);
		ez = (Setpoint.z - odom.pose.pose.position.z);
		znow =odom.pose.pose.position.z;

		Pose.z_translational = odom.pose.pose.position.z;
		eyaw = Setpoint.yaw - yaw_;

		// Store errors for later reference
		errorsx_.push_back(ex);
		errorsy_.push_back(ey);
		errorsz_.push_back(ez);
		errorsyaw_.push_back(ey);

		zprevious_.push_back(znow);

		// Do some memory management
		if (times_.size() > error_memory_) {
			errorsx_.erase(errorsx_.begin());
			errorsy_.erase(errorsy_.begin());
			errorsz_.erase(errorsz_.begin());
			errorsyaw_.erase(errorsyaw_.begin());
			zprevious_.erase(zprevious_.begin());
			times_.erase(times_.begin());
			yaws_.erase(yaws_.begin());
		}

		// Store speeds for velocity damping
		speeds_.push_back(odom.twist.twist);
		if (speeds_.size() > 15) {
			speeds_.erase(speeds_.begin());
		}
		speed_ = odom.twist.twist;

		// Save definite error
		Error.x_translational = ex;
		Error.y_translational = ey;
		Error.z_translational = ez;
		Error.z_rotational = eyaw;

		// Save to store for I action. For some wicked reason, we have to look two frames back
		if (i_action_) {
			int n = times_.size();
			isum.x = isum.x + 0.5 * ex * (times_[n] - times_[n - 2]);
			isum.y = isum.y + 0.5 * ey * (times_[n] - times_[n - 2]);
		} else {
			isum.x = 0;
			isum.y = 0;
		}

		// Get derivative of the error by using a fixed order fixed window approach
		ErrorDot.x_translational = fofw(errorsx_, times_, 6);
		ErrorDot.y_translational = fofw(errorsy_, times_, 6);
		ErrorDot.z_translational = fofw(errorsz_, times_, 6);
		ErrorDot.z_rotational = fofw(errorsyaw_, times_, 10);

		std_msgs::Float32 errord_msg;
		errord_msg.data = ErrorDot.x_translational;
		errordx_publisher_.publish(errord_msg);

		// If hovermode is enabled, and you are within the treshold, publish hover message
		if (hovermode_) {
			if (std::abs(Error.x_translational) < hover_treshold_
					&& std::abs(Error.y_translational) < hover_treshold_
					&& std::abs(Error.z_translational) < hover_treshold_) {
				Error.x_translational = 0;
				Error.y_translational = 0;
				Error.z_translational = 0;
				Error.z_rotational = 0;

				ErrorDot.x_translational = 0;
				ErrorDot.y_translational = 0;
				ErrorDot.z_translational = 0;
				ErrorDot.z_rotational = 0;
				ROS_INFO("HOVERING");
			}
		}

		// Publish once every x ms, based on the frequency.
		if (current_time - previous_publish_time_ > 1 / freq_) {
			publishControl();
			previous_publish_time_ = current_time;
		}

	}

	/*
	 * simulationCallback
	 *
	 * Function called when the topic /gazebo/model_state changes.
	 * This function does all the error calculations regarding
	 * the position of the simulated vehicle and the desired position.
	 * The same for the simulated vehicle velocity and desired velocity.
	 * This topic /gazebo/model_state comes from the gazebo simulation node.
	 *
	 * @param			<gazebo_msgs::ModelStates> odom from topic /gazebo/model_state.
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	void simulationCallback(const gazebo_msgs::ModelStates pose) {
		// This function is called whenever a message on the simulation channel is found

		// Calculate roll pitch and yaw from quaternion data
		tf::Quaternion q(pose.pose[11].orientation.x,
				pose.pose[11].orientation.y, pose.pose[11].orientation.z,
				pose.pose[11].orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		// Rotation error

		yaw_ = yaw;
		Error.z_rotational = Setpoint.yaw - yaw;

		std_msgs::Float32 yaw_msg;
		yaw_msg.data = yaw_;
		yaw_publisher_.publish(yaw_msg);

		// Position error
		Error.x_translational = -(Setpoint.x - pose.pose[11].position.x);
		Error.y_translational = -(Setpoint.y - pose.pose[11].position.y);
		Error.z_translational = Setpoint.z - pose.pose[11].position.z;

		publishControl();

		//ROS_INFO("Setpoint %f %f %f", Setpoint.x, Setpoint.y, Setpoint.z);
		ROS_INFO("Error %f %f %f %f", Error.x_translational,
				Error.y_translational, Error.z_translational, yaw);

	}

	/*
	 * publishControl
	 *
	 * This function relates all errors in absolute world frame to body actions.
	 * Positive errors means that we need positive action in that direction.
	 *
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	void publishControl() {

		std_msgs::Float32 errorx_msg;
		errorx_msg.data = Error.x_translational;
		errorx_publisher_.publish(errorx_msg);

		/* Start off with nonlinear coeff on P gain
		 If the error is positive, a positive nonlin* will result in additional gain.
		 */
		float addgainx, addgainy, addgainz;
		addgainx = 0;
		addgainy = 0;
		addgainz = 0;

		// P - action
		double px, py, pz_new, pyaw, pz;
		px = Error.x_translational * (Gains.p_translational * (1 + addgainx));
		py = Error.y_translational * (Gains.p_translational * (1 + addgainy));
		pz_new = Error.z_translational * (Gains.p_z * (1 + addgainz))
				* (Pose.z_translational) * G_CONSTANT;
		pz = Error.z_translational * (Gains.p_z * (1 + addgainz));
		pyaw = Error.z_rotational * Gains.p_rotational;

		std_msgs::Float32 p_action;
		p_action.data = px;
		p_publisher_.publish(p_action);

		// D - action
		double dx, dy, dz, dyaw,dz_new;
		dx = ErrorDot.x_translational * Gains.d_translational;
		dy = ErrorDot.y_translational * Gains.d_translational;
		dz = ErrorDot.z_translational * Gains.d_z;
		dz_new = -speed_.linear.z* Gains.d_z;
		dyaw = ErrorDot.z_rotational * Gains.d_rotational;

		std_msgs::Float32 d_action;
		d_action.data = dx;
		d_publisher_.publish(d_action);

		double vx, vy, vz;
		vx = 0;
		vy = 0;
		vz = 0;
		// Make it go through honey!
		if (velocity_damping_) {
			vx = speeds_.back().linear.x * Gains.velocity;
			vy = speeds_.back().linear.y * Gains.velocity;
			vz = speeds_.back().linear.z * Gains.velocity;
		}

		std_msgs::Float32 vel_action;
		vel_action.data = vx;
		velocity_damping_publisher_.publish(vel_action);

		// If I action is present and enable
		double ix, iy;
		if (i_action_) {
			ix = isum.x * Gains.i;
			iy = isum.y * Gains.i;
		} else {
			ix = 0;
			iy = 0;
		}

		// Total required actions in world frame
		double wx, wy, wz, wyaw;
		wx = px + dx + vx + ix;
		wy = py + dy + vy + iy;
		wz = pz + dz + vz;
		wyaw = pyaw + dyaw;

		// Transformation to the body fixed frame
		// This can differ between simulation and real life
		double qx, qy, qz, qyaw;
		if (simulation) {
			qx = wx * -cos(yaw_) + wy * -sin(yaw_);
			qy = wx * sin(yaw_) + wy * -cos(yaw_);
		} else {
			qx = wx * -cos(yaw_) + wy * -sin(yaw_);
			qy = wx * sin(yaw_) + wy * -cos(yaw_);
		}
		qz = wz;
		qyaw = wyaw;

		//Cees
//		velocity_.linear.x = qx;
//		velocity_.linear.y = qy;
//		velocity_.linear.z = qz;
//		velocity_.angular.z = qyaw;
		//Lucas
		velocity_.linear.x = -qx;
		velocity_.linear.y = -qy;
		velocity_.linear.z = qz;		//mgh?
		velocity_.angular.z = -qyaw;

		// LIMIT output to 1
		if (velocity_.linear.x > 1) {
			velocity_.linear.x = 1;
		}
		if (velocity_.linear.y > 1) {
			velocity_.linear.y = 1;
		}
		if (velocity_.linear.z > 1) {
			velocity_.linear.z = 1;
		}
		if (velocity_.linear.x < -1) {
			velocity_.linear.x = -1;
		}
		if (velocity_.linear.y < -1) {
			velocity_.linear.y = -1;
		}
		if (velocity_.linear.z < -1) {
			velocity_.linear.z = -1;
		}

		if (velocity_.angular.x > 1) {
			velocity_.angular.x = 1;
		}
		if (velocity_.angular.y > 1) {
			velocity_.angular.y = 1;
		}
		if (velocity_.angular.z > 1) {
			velocity_.angular.z = 1;
		}
		if (velocity_.angular.x < -1) {
			velocity_.angular.x = -1;
		}
		if (velocity_.angular.y < -1) {
			velocity_.angular.y = -1;
		}
		if (velocity_.angular.z < -1) {
			velocity_.angular.z = -1;
		}

		std_msgs::Float32 tot_x;
		tot_x.data = velocity_.linear.x;
		x_publisher_.publish(tot_x);
		//ROS_INFO("before publish");

		velocity_publisher_.publish(velocity_);
	}

	/*
	 * lowPassFilter
	 *
	 * Extremely simple filter.
	 *
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	double lowPassFilter(double x, double y0, double dt, double T) {
		double res = y0 + (x - y0) * (dt / (dt + T));
		return res;
	}

	/*
	 * fofw
	 *
	 * First order fixed window approach to get the derivative.
	 *
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	double fofw(std::vector<double> memory, std::vector<double> times,
			int samplesBack) {
		int n = times.size();
		if (n > samplesBack) {
			return (memory[n - 1] - memory[n - 1 - samplesBack])
					/ (times[n - 1] - times[n - 1 - samplesBack]);
		} else if (n > 1) {
			return (memory[n - 1] - memory[0]) / (times[n - 1] - times[0]);
		}
	}

	/*
	 * avg
	 *
	 * Get the average from a set of values.
	 *
	 * @author  		Cees Trouwborst <ceestrouwborst@gmail.com>
	 * @date    		JLY 2014
	 */
	double avg(std::vector<double> memory, int samplesBack) {
		if (memory.size() > samplesBack) {
			double s;
			s = 0;
			for (int n = 0; n < samplesBack; n++) {

				s = s + memory[memory.size() - n - 1];

			}

			return s / samplesBack;
		} else {
			return memory.back();
		}

	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller");
	Control control;

	/*
	 dynamic_reconfigure::Server<controller::ParamsConfig> srv;
	 dynamic_reconfigure::Server<controller>::CallbackType f;
	 f = boost::bind(&control::dynConfCb, &control, _1, _2);
	 srv.setCallback(f);*/

	ros::spin();
	ros::Rate loop_rate(10);
	loop_rate.sleep();
	return 0;
}

