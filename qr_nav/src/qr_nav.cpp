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

 // Target goals[4];// = {(0.0, 0.0), (0.0, 4*QRDISTANCE), (11*QRDISTANCE, 4*QRDISTANCE), (11*QRDISTANCE, 0.0)};
//  goals[0] = {0.0, 0.0};
//  goals[1] = {0.0, 4*QRDISTANCE};
//  goals[2] = {11*QRDISTANCE, 4*QRDISTANCE};
//  goals[3] = {11*QRDISTANCE, 0.0};

  //============= motion 1 ================//
  /*#5          #3
   * ------------------------#5
   *            |
   *            |
   *            |
   *            |
   *            |
   *            |
   *            |
   *          #3-----------------#1
   *
   *
   * */
/*  Target goals[4];
  goals[0].x = 0.0;
  goals[0].y = 0.0;
  goals[1].x = 0.0;
  goals[1].y = 4*QRDISTANCE;
  goals[2].x = 11*QRDISTANCE;
  goals[2].y = 4*QRDISTANCE;

  goals[3].x = 11*QRDISTANCE;
  //goals[3].y = 0.0;
  goals[3].y = 9*QRDISTANCE;*/

  //=========================================//


  //============= motion 2 ================//
  /*#5          #3
   * ------------------------#5
   *            |
   *            |
   *            |
   *            |
   *            |
   *            |
   *            |
   * --------------------------#1
   *#5         #3
   *
   * */

/*
  Target goals[12];
  goals[0].x = 0.0;
  goals[0].y = 0.0;

  goals[1].x = 0.0;
  goals[1].y = 9*QRDISTANCE;

  goals[2].x = 0.0;
  goals[2].y = 4*QRDISTANCE;

  goals[3].x = 11*QRDISTANCE;
  goals[3].y = 4*QRDISTANCE;

  goals[4].x = 11*QRDISTANCE;
  goals[4].y = 0.0;

  goals[5].x = 11*QRDISTANCE;
  goals[5].y = 9*QRDISTANCE;

  goals[6].x = 11*QRDISTANCE;
  goals[6].y = 4*QRDISTANCE;

/*
 * /*
  goals[7].x = 0.0;
  goals[7].y = 4*QRDISTANCE;

  goals[8].x = 0.0;
  goals[8].y = 0.0;
*/

/*
  //添加了路径.
  goals[7].x = -6*QRDISTANCE;
  goals[7].y = 4*QRDISTANCE;

  goals[8].x = -6*QRDISTANCE;
  goals[8].y = 9*QRDISTANCE;

  goals[9].x = -6*QRDISTANCE;
  goals[9].y = 4*QRDISTANCE;

  goals[10].x = 0.0;
  goals[10].y = 4*QRDISTANCE;

  goals[11].x = 0.0;
  goals[11].y = 0.0;

  */

  //演示
/*  Target goals[8];
  goals[0].x = 0.0;
  goals[0].y = 0.0;

  goals[1].x = 0.0;
  goals[1].y = -8*QRDISTANCE;

  goals[2].x = -22*QRDISTANCE;
  goals[2].y = -8*QRDISTANCE;

  goals[3].x = -12*QRDISTANCE;
  goals[3].y = -8*QRDISTANCE;

  goals[4].x = -12*QRDISTANCE;
  goals[4].y = 2*QRDISTANCE;

  goals[5].x = -12*QRDISTANCE;
  goals[5].y = -8*QRDISTANCE;

  goals[6].x = 0.0;
  goals[6].y = -8*QRDISTANCE;

  goals[7].x = 0.0;
  goals[7].y = 0.0;*/

  //论文实验矩形路径
    Target goals[6];
    goals[0].x = 0.0;
    goals[0].y = 0.0;

    goals[1].x = 0.0;
    goals[1].y = -8*QRDISTANCE;

    goals[2].x = -12*QRDISTANCE;
    goals[2].y = -8*QRDISTANCE;

    goals[3].x = -12*QRDISTANCE;
    goals[3].y = 2*QRDISTANCE;

    goals[4].x = 0.0;
    goals[4].y = 2*QRDISTANCE;

    goals[5].x = 0.0;
    goals[5].y = 0.0;
  std::vector<Target> goals_vec;
  unsigned int len = sizeof(goals)/sizeof(Target);      //求结构体对象数组的长度.

  for(unsigned int i = 0; i < len; i++)
  {
    goals_vec.push_back(goals[i]);
  }
  //=========================================//
  PoseAlign align(goals_vec);


  ros::spin();
  return 0;
}


