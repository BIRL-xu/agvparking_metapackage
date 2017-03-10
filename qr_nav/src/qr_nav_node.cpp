/*
 * qr_nav_node.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: paul
 */

#include "qr_nav/error_align.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "qr_nav_node");
  ros::NodeHandle n;
//  const float goals[] = {0.0, 4*QRDISTANCE, 8*QRDISTANCE, 4*QRDISTANCE, 8*QRDISTANCE, 0.0};

  const Target goals[] = {(0.0, 0.0), (0.0, 4*QRDISTANCE), (8*QRDISTANCE, 4*QRDISTANCE), (8*QRDISTANCE, 0.0)};

  PoseAlign align(goals, sizeof(goals)/sizeof(float));


  ros::spin();
  return 0;
}


