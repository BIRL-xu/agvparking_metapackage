/*
 * pose_ekf_node.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: paul
 */

#include "pose_ekf/pose_ekf.h"

int main(int argc, char *argv[])
{
  tf::Matrix3x3 m(1, 2, 3,
                  4, 5, 6,
                  7, 8, 9);
  tf::Vector3 co(m.getRow(0));
  tf::Vector3 c1(m[0]);
  tf::Vector3 c2(m[1]);
  tf::Vector3 c3(m[3]);
  ros::init(argc, argv, "pose_ekf_node");

  PoseEKF ekf;
  ros::spin();

  return 0;
}


