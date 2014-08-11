/*!
 * \file   HapticTeleopSimulinkMap.h
 *
 * \date   Jun 24, 2011
 * \author Rueesch Andreas
 */

#ifndef HAPTICTELEOPSIMULINKMAP_H_
#define HAPTICTELEOPSIMULINKMAP_H_

// ------ INCLUDES ------
//#include "HapticTeleop.h"
//#include "HapticUdpBridgeXmit.h"
#include "UdpBridgeRecv.h"

// ------ DEFINES -------

// Networking
#define UDP_IP             "127.0.0.1" //     "192.168.0.103" //       // Target IP
#define UDP_PORT_XMIT       12121                  // Port no.
#define UDP_PORT_RECV       12122                  // Port no.

#define MAX_DELTA_TIME      5                      // in [sec]

//! Haptic telemanipulation of an UAV, running the algorithms in MATLAB
/*!
 *  This mode allows to use MATLAB/Simulink to derive/run the teleoperation
 *  algorithms, for this purpose a UDP bridge is established.
 *
 *  \author     Rueesch Andreas
 *  \date       June, 2011
 *  \version    0.2
 */
class TeleopSimulinkMap : public Teleop
{
public:
  //! class constructor
  TeleopSimulinkMap(ros::NodeHandle &nh);

  //! class destructor
  virtual ~TeleopSimulinkMap();

  //! mapping of the  state to a desired position reference
  virtual void BuildPosReference(
      const teleop_msgs::State::ConstPtr& haptic_state);

  //! mapping of the haptic state to a desired position reference
 // virtual void BuildPosReference_falcon(
      //const hapticteleop_msgs::HapticState::ConstPtr& haptic_state_falcon);

  //! stores current uav pose, (force feedback is calculated in simulink)
  void BuildForceFeedback(
      const nav_msgs::Odometry::ConstPtr& uav_pose);

private:
  //! Udp bridge to MATLAB - send
  //UdpBridgeXmit UdpBridgeXmit;

  //! Udp bridge to MATLAB - receive
  UdpBridgeRecv UdpBridgeRecv;
};

#endif /* HAPTICTELEOPSIMULINKMAP_H_ */
