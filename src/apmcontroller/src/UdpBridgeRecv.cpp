/*!
 * \file     UdpBridgeRecv.cpp
 *
 * \author   Rueesch Andreas
 * \date     Jul 14, 2011
 * \version  0.1
 */

#include "UdpBridgeRecv.h"

UdpBridgeRecv::UdpBridgeRecv()
{
  // TODO Auto-generated constructor stub

}

UdpBridgeRecv::~UdpBridgeRecv()
{
  // TODO Auto-generated destructor stub
  close(mSockFd);
}

/**
 * Init
 *
 *init udp port
 *
 * @author  Janosch Nikolic <janosch.nikolic@mavt.ethz.ch>, Michael Burri
 * @version 0.2
 * @date    APR 2011
 */
void UdpBridgeRecv::Init(const char *ipadr, int port)
{
  mIp=ipadr;
  mPort=port;

  mSockFd=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

  mCliAddr.sin_family=AF_INET;
  mCliAddr.sin_addr.s_addr=inet_addr(mIp);
  mCliAddr.sin_port=htons(mPort);
  mCliAddrLen=sizeof(struct sockaddr_in);

  int bindRet=bind(mSockFd,(struct sockaddr*)&mCliAddr,sizeof(mCliAddr));

  if(bindRet < 0)
  {
    ROS_ERROR("COULD NOT BIND SPECIFIED PORT - NOT BRIDGING.");
    return;
  }

  // set udp call to non blocking
  fcntl(mSockFd, F_SETFL, O_NONBLOCK);

  // init char buffer
  for(int i=0;i<NUM_RX_FLOATS*4;i++)
  {
    CharBuffer[i]=0;
  }

  // set float pointer to char buffer
  FloatBuffer = (float*)CharBuffer;


  ROS_INFO("[ --- udpbridge_teleop --- ] UDP SERVER ON IP %s PORT %d : Bridging"
           " from MATLAB to ROS.", mIp, mPort);
}

/**
 * GetData waits for data on the udp socket and returns false if timeout was reached or message not correctly received
 *
 * @author  Michael Burri
 * @version 0.2
 * @date    MAR 2011
 */

bool UdpBridgeRecv::GetData()
{
  // set timeout of udp call
  struct timeval tv;
  tv.tv_sec = RX_TIMEOUT;
  tv.tv_usec = 0;

  // set socket that has to be monitored
  FD_ZERO(&mRfds);
  FD_SET(mSockFd, &mRfds);

  // check socket for new data
  if(!select(mSockFd + 1, &mRfds, NULL, NULL, &tv))
  {
    ROS_WARN("[ --- udpbridge_teleop --- ] UDP timeout reached");
    return false;
  }
//fprintf(stderr,"transmitting pos: %f %f %f\n", FloatBuffer[0], FloatBuffer[1], FloatBuffer[2]);

  // receive message from udp
  int rcvd_bytes = recvfrom(mSockFd,CharBuffer,NUM_RX_FLOATS*4,0,
                            (struct sockaddr*)&mCliAddr,
                            (socklen_t*)&mCliAddrLen);
  // check if message has correct size
  if(rcvd_bytes<NUM_RX_FLOATS*4)
  {
    ROS_ERROR("[ --- udpbridge_teleop --- ] UDP package to small");
    return false;
  }

  return true;
}















