/*
 * pose_align.h
 *
 *  Created on: Jan 11, 2017
 *      Author: paul
 */

#ifndef INCLUDE_QR_NAV_POSE_ALIGN_H_
#define INCLUDE_QR_NAV_POSE_ALIGN_H_
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include "agvparking_msg/AgvOdom.h"
#include <tf/transform_broadcaster.h>
#include <vector>
#define SIGN(x) ((x > 0) - (x < 0))
#define ALIGN_VEL 0.02
#define TRAVEL_VEL_X 0.35
#define INIT_VEL 0.2
void waitMilli(int milliseconds)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds(milliseconds));
}

class PoseAlign
{
public:
  PoseAlign();
  ~PoseAlign();
private:
  struct QRPose
  {
    unsigned short int tag_num;
    float x;    //m.
    float y;    //m.
    float angle;        //rad.
  }qr_pose_;
  struct PoseError
  {
    unsigned short int tag_num;
    float x;    //m.
    float y;    //m.
    float angle;//m.
  }error_;
  struct MoveState
  {
    std::string moving_dir;     //"":no; "X+";"X-";"Y+";"Y-".
    bool start;         //
    bool speedup;       //true:加速；false:减速。
    std::string expeted_dir;   //"":no; "X+";"X-";"Y+";"Y-".
    MoveState()
    {
      moving_dir = "";
      start = 1;
      expeted_dir = "Y+";
      speedup = true;
    }

  }motion_state_;
  void qrInfoCallback(const agvparking_msg::AgvOdom::ConstPtr &qr_info);
  void getPoseError();
  void alignAndMove();
  void stop();
  void alignAlgorithm(const QRPose &error, float &vx, float &vy, float &vw);
  void alignInit(const QRPose &error);
  float angleMapAndRad(const float &angle);
  void setState(const MoveState &state);
  MoveState getState();
private:

  //ros variable.
  ros::NodeHandle nh_;
  ros::Subscriber QR_odom_sub_;         //订阅自定义的二维码里程计。
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist cmd_vel_;
  //varibles.
  float pose3f_[3];
  float vel3f_[3];
  bool aligning_, x_aligning_, y_aligning_, yaw_aligning_;
  bool moving_;
  float position_tolerance_, angle_tolerance_;

  //boost mutex
  boost::mutex tag_mutex_, error_mutex_, state_mutex_;
};




#endif /* INCLUDE_QR_NAV_POSE_ALIGN_H_ */
