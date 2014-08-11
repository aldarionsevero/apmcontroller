/*!
 * \file     UdpBridgeRecv.h
 *
 * \author   Rueesch Andreas
 * \date     Jul 14, 2011
 * \version  0.1
 *
 * This UDP bridge is based on the air_udpbridge_recv class by ASL/ETHZ
 */

#ifndef UDPBRIDGERECV_H_
#define UDPBRIDGERECV_H_


// ------ INCLUDES ------
#include "ros/ros.h"

// ------ NETWORKING ----
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>


// ------ MESSAGE -------
#define BUFFLEN             512     // buffer size

#define NUM_RX_FLOATS         4     // Number of 32bit little endian floats to
                                    // be transmitted over UDP.

#define UDP_RX_POS_CMD_X      0
#define UDP_RX_POS_CMD_Y      1
#define UDP_RX_POS_CMD_Z      2
#define UDP_RX_POS_CMD_YAW    3

#define UDP_RX_FF_X           4
#define UDP_RX_FF_Y           5
#define UDP_RX_FF_Z           6

// ------ GLOBAL DEF -------
#define RX_TIMEOUT            1     // [sec] to wait for a UDP msg

// ------ CLASS DECLARATION ------

//! UDP bridge to send the state of the  device to MATLAB/Simulink
/*!
 *  This class is based on the UDP bridge of the AIR_control project of ETHZ.
 *
 *  \author     Rueesch Andreas
 *  \date       June, 2011
 *  \version    0.2
 */
class UdpBridgeRecv
{
protected:
  struct sockaddr_in mCliAddr;
  int mCliAddrLen;
  int mSockFd;
  int mPort;
  const char *mIp;

  fd_set mRfds;

public:
  //! class constructor
  UdpBridgeRecv();

  //! class destructor
  virtual ~UdpBridgeRecv();

  //! initializes the udp port
  void Init(const char *ipadr, int port);

  //! gets data from the udp port
  bool GetData();

  //! receive buffer
  char CharBuffer[BUFFLEN];
  float *FloatBuffer;
};

#endif /* UDPBRIDGERECV_H_ */
