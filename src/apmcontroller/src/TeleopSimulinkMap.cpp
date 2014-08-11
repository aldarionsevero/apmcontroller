/*!
 * \file   HapticTeleopSimulinkMap.cpp
 *
 * \date   Jun 24, 2011
 * \author Rueesch Andreas
 */

#include "air_hapticteleop/HapticTeleopSimulinkMap.h"

//bool x=true;

//! class constructor
/*!
 * \param   ros::NodeHandle& a ros node handle
 *
 * \author  Rueesch Andreas
 * \date    Jun 24, 2011
 * \version 0.3
 */
TeleopSimulinkMap::TeleopSimulinkMap(ros::NodeHandle &nh) : Teleop(nh)
{
  ROS_INFO("---  Teleoperation is running in MATLAB/Simulink mode ---");

  // init UDP bridge for simulink mode
  //UdpBridgeXmit.Init(UDP_IP, UDP_PORT_XMIT);
  UdpBridgeRecv.Init(UDP_IP, UDP_PORT_RECV);
}

//! class destructor
TeleopSimulinkMap::~TeleopSimulinkMap()
{
  // TODO Auto-generated destructor stub
  // TODO close udp ports ~> call bridge destructor
}

/*!
 * sends the current state of the  device plus of the uav through a UDP
 * bridge to MATLAB/Simulink, and publishes the received position reference and
 * force feedback to ros.
 *
 * \param     _state state of the hapitc device
 *
 * \note      current uav state has to be up-to-date
 * \author    Rueesch Andreas
 * \date      Jul 19, 2011
 * \version   0.3
 */
void TeleopSimulinkMap::BuildPosReference(
    const hapticteleop_msgs::HapticState::ConstPtr& haptic_state)
{
  // check time duration between now and last uav state
//  if((ros::Time::now().toSec() - CurrentUavPose.header.stamp.toSec())
//      <= MAX_DELTA_TIME)
//  {
//    // enable force feedback if force button is enabled
//    if( haptic_state->button2.data )
//    {
//      FF_ENABLED = true;
//    }
//    else
//    {
//      FF_ENABLED = false;
//    }
//
//    // send state of haptic device and uav
//    UdpBridgeXmit.SendStates( haptic_state, CurrentUavPose );
//     //UdpBridgeXmit.SendStates( haptic_state, haptic_state_falcon, CurrentUavPose );
//
//    // receive pos reference
//    float *tmp_buff = UdpBridgeRecv.FloatBuffer;
//
//#ifdef __EXP_ANU__
//      // initialize pos reference
//      hapticteleop_msgs::HapticPosRef haptic_pos_ref;
//
//      // set the header information
//      haptic_pos_ref.header.stamp = ros::Time::now();
//
//      // fill msg with context of the buffer
//      haptic_pos_ref.ref_pos.x = tmp_buff[UDP_RX_POS_CMD_X];
//      haptic_pos_ref.ref_pos.y = tmp_buff[UDP_RX_POS_CMD_Y];
//      haptic_pos_ref.ref_pos.z = tmp_buff[UDP_RX_POS_CMD_Z];
//      haptic_pos_ref.ref_yaw.data = tmp_buff[UDP_RX_POS_CMD_YAW];
//
//      // send haptic reference
//      SendPosReference(haptic_pos_ref, haptic_state->button2.data);
//
//      // init force feedback
//      geometry_msgs::Vector3 ForceFeedback;
//      ForceFeedback.x = 0.0;
//      ForceFeedback.y = 0.0;
//      ForceFeedback.z = 0.0;
//
//      if ( FF_ENABLED )
//      {
//        // fill vector with context of the buffer
//        if ( FF_ENABLED_X )
//        {
//          ForceFeedback.x = tmp_buff[UDP_RX_FF_X];
//        }
//        if ( FF_ENABLED_Y )
//        {
//          ForceFeedback.y = tmp_buff[UDP_RX_FF_Y];
//        }
//        if ( FF_ENABLED_Z )
//        {
//          ForceFeedback.z = tmp_buff[UDP_RX_FF_Z];
//        }
//      }
//      // publish force feedback
//      PubForceFeedback.publish(ForceFeedback);
//  }
//#else
   //if(x==false)
   if(UdpBridgeRecv.GetData())
    {
      // initialize pos reference
      hapticteleop_msgs::HapticPosRef haptic_pos_ref;

      // set the header information
      haptic_pos_ref.header.stamp = ros::Time::now();

      // fill msg with context of the buffer
      haptic_pos_ref.ref_pos.x = tmp_buff[UDP_RX_POS_CMD_X];
	//fprintf(stderr,"ref=%.3lf\n",haptic_pos_ref.ref_pos.x);
      haptic_pos_ref.ref_pos.y = tmp_buff[UDP_RX_POS_CMD_Y];
	//fprintf(stderr,"ref=%.3lf\n",haptic_pos_ref.ref_pos.y);
      haptic_pos_ref.ref_pos.z = tmp_buff[UDP_RX_POS_CMD_Z];
	//fprintf(stderr,"ref=%.3lf\n",haptic_pos_ref.ref_pos.z);
      haptic_pos_ref.ref_yaw.data = tmp_buff[UDP_RX_POS_CMD_YAW];
	//fprintf(stderr,"ref=%.3lf\n",haptic_pos_ref.ref_yaw.data);

      // send haptic reference
      SendPosReference(haptic_pos_ref, haptic_state->button2.data);

      // init force feedback
      geometry_msgs::Vector3 ForceFeedback;
      ForceFeedback.x = 0.0;
      ForceFeedback.y = 0.0;
      ForceFeedback.z = 0.0;

      if ( FF_ENABLED )
      {
        // fill vector with context of the buffer
        if ( FF_ENABLED_X )
        {
          ForceFeedback.x = tmp_buff[UDP_RX_FF_X];
        }
        if ( FF_ENABLED_Y )
        {
          ForceFeedback.y = tmp_buff[UDP_RX_FF_Y];
        }
        if ( FF_ENABLED_Z )
        {
          ForceFeedback.z = tmp_buff[UDP_RX_FF_Z];
        }
      }

      // publish force feedback
      PubForceFeedback.publish(ForceFeedback);
    }
  }
  else
  {
   ROS_WARN("current pose estimation too old!");

//newly added abeje for ANU experiment
  
/*// init force feedback

   // initialize pos reference
      hapticteleop_msgs::HapticPosRef haptic_pos_ref;

      // set the header information
      haptic_pos_ref.header.stamp = ros::Time::now();

    // fill msg with context of the buffer
      haptic_pos_ref.ref_pos.x = 0;
      haptic_pos_ref.ref_pos.y = 0;
      haptic_pos_ref.ref_pos.z = 0
      haptic_pos_ref.ref_yaw.data = 0;


      // send haptic reference
      SendPosReference(haptic_pos_ref, haptic_state->button2.data);


*/
      geometry_msgs::Vector3 ForceFeedback;
      ForceFeedback.x = 0.0;
      ForceFeedback.y = 0.0;
      ForceFeedback.z = 0.0;

      // publish force feedback
      PubForceFeedback.publish(ForceFeedback);

  } 

#endif
}

/*!
 * saves the current uav state for the UDP bridge to MATLAB/Simulink, nothing
 * is published here!
 *
 * \param     uav_state state of the uav
 *
 * \author    Rueesch Andreas
 * \date      Jul 19, 2011
 * \version   0.3
 */
void TeleopSimulinkMap::BuildForceFeedback(
    const nav_msgs::Odometry::ConstPtr& uav_pose)
{
  // current time stamp
  CurrentUavPose.header.stamp = ros::Time::now();

  // save current UAV pose
  CurrentUavPose.pose.pose = uav_pose->pose.pose;

  // save current UAV velocity
#ifndef __EXP_ETH__
  CurrentUavPose.twist.twist.linear = UavVelEstimator;
#else
  CurrentUavPose.twist.twist = uav_pose->twist.twist;
#endif

#ifdef __EXP_ETH__
    // conv to NED !!
    CurrentUavPose.pose.pose.position.y = - CurrentUavPose.pose.pose.position.y;
    CurrentUavPose.pose.pose.position.z = - CurrentUavPose.pose.pose.position.z;
    CurrentUavPose.twist.twist.linear.y = - CurrentUavPose.twist.twist.linear.y;
    CurrentUavPose.twist.twist.linear.z = - CurrentUavPose.twist.twist.linear.z;
#endif
}
