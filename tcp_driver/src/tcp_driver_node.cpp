/*
 * tcp_driver_node.cpp
 *
 *  Created on: Dec 9, 2016
 *      Author: paul
 */

#include "tcp_driver/tcp_driver.h"



int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tcp_driver_node");
  ros::NodeHandle n;
  boost::asio::io_service io_sev;
  unsigned int port = 3000;
  ServerSocket sever(io_sev, port);

  ros::spin();
  return 0;
}


