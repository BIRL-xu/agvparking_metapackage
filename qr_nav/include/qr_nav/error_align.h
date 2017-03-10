/*
 * error_align.h
 *
 *  Created on: Jan 12, 2017
 *      Author: paul
 */

#ifndef INCLUDE_QR_NAV_ERROR_ALIGN_H_
#define INCLUDE_QR_NAV_ERROR_ALIGN_H_
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "agvparking_msg/AgvOdom.h"
#include <tf/transform_broadcaster.h>
#include "ros/timer.h"

#define SIGN(x) ((x > 0) - (x < 0))
#define ALIGN_VEL       0.01
#define START_VEL       0.1
#define TRAVEL_VEL_X    0.4
#define TRAVEL_VEL_Y    0.3
#define OMEGA           0.15

#define QRDISTANCE      0.5     //QR码的间距，默认50cm.
#define TRAVEL_ERROR    0.005      //行驶过程中，每边的误差允许范围。
#define PREDISTANCE     0.35    //估计修正预苗距离。

void waitMilli(int milliseconds)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds(milliseconds));
}
struct Target
{
  double x;
  double y;
};

class PoseAlign
{
public:

  PoseAlign(const Target goal[], long unsigned int size);
  ~PoseAlign();
private:
  struct QRPose
  {
    unsigned short int tag_num;
    float x;    //m.
    float y;    //m.
    float angle;        //rad.
    QRPose()
    {
      tag_num = 0;
      x = 0.0;
      y = 0.0;
      angle = 0.0;
    }
  }qr_pose_;

  struct OdomData
  {
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float vw;
    float lift; //抬升。
    OdomData()
    {
      x = 0.0;
      y = 0.0;
      yaw = 0.0;
      vx = 0.0;
      vy = 0.0;
      vw = 0.0;
      lift = 0.0;
    }
  }odom_data_;

  struct QROdom
  {
    QRPose pose;
    OdomData odom;
  }qr_odom_;
  struct MotionState
  {
    std::string moving_dir;     //"":no; "X+";"X-";"Y+";"Y-".
    std::string expeted_dir;   //"":no; "X+";"X-";"Y+";"Y-".
    char start;         //0：none;1：起点；2:终点。
    char speedup;       //0：none;1:加速；-1:减速。
    bool lift;
    MotionState()
    {
      moving_dir = "";
      expeted_dir = "";
      start = 1;
      speedup = 0;
      lift = false;
    }
  }motion_state_;

  struct Error
  {
    float x;
    float y;
    float yaw;
    Error()
    {
      x = 0.0;
      y = 0.0;
      yaw = 0.0;
    }
  }err_;




  void qrInfoCallback(const agvparking_msg::AgvOdom::ConstPtr &qr_info);
  void getPoseError();
  void alignAndMove();
  void move();
  void alignAlgorithm(const QROdom &qr_odom, const Error &error, float &vx, float &vy, float &vw, unsigned short int &t);
  void stop();
  void alignInit(const Error &error, geometry_msgs::Twist &cmd_vel);
  void trajAlign(const QROdom &qr_odom, const Error &error, geometry_msgs::Twist &vel, float dist);
  void trajAlign(const Error &error, const Error &init_error, geometry_msgs::Twist &vel);
  float angleMapAndRad(const float &angle);
  void setState(const MotionState &state);
  MotionState getState();
  bool getCmdVel(const Target &goal, const QROdom &cur_pose, geometry_msgs::Twist& vel);

  void setGoals(const Target goal[], const short int size);
private:

  //ros variable.
  ros::NodeHandle nh_;
  ros::Subscriber QR_odom_sub_;         //订阅自定义的二维码里程计。
  ros::Publisher cmd_vel_pub_, combined_odom_pub_;
  geometry_msgs::Twist cmd_vel_, last_cmd_;
  ros::WallTime last_time_;             //世界时钟时间。
  ros::Time time_;
  //varibles.
  float pose3f_[3];
  float vel3f_[3];
  float odom_ref_;      //作为里程计参考的数据。
  QROdom ref_;
  float d_const_;
  bool aligning_, aligned_, x_aligning_, y_aligning_, yaw_aligning_, traj_aligning_;
  bool moving_;
  bool navigation_, goback_;
  bool time_update_;
  float position_tolerance_, angle_tolerance_;
  agvparking_msg::AgvOdom com_odom_msg_;
  //boost mutex
  boost::timed_mutex  odom_ref_mutex_, error_mutex_, state_mutex_;
  boost::condition_variable_any cond_;
  boost::shared_mutex odom_mutex_;              //共享锁。

 //error
  Error InitErr_;

  Target *goal_;
  long unsigned int goals_num_;
};
#endif /* INCLUDE_QR_NAV_ERROR_ALIGN_H_ */
