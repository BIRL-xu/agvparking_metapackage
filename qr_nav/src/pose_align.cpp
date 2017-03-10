/*
 * pose_align.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: paul
 */
#include "qr_nav/pose_align.h"



PoseAlign::PoseAlign():nh_(ros::NodeHandle("~")),aligning_(false),x_aligning_(false),y_aligning_(false),yaw_aligning_(false)
{
  memset(pose3f_ ,0 , 3);
  memset(vel3f_ ,0 , 3);
  moving_ = false;
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.angular.z = 0.0;
  position_tolerance_ = 0.005;   //5mm.
  angle_tolerance_ = 0.1;        //0.1rad=5.73°.
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 50);    //"/cmd_vel"为全局名称，"cmd_vel"为局部名称，发布时自动加上节点名空间，如/qr_nav_node/cmd_vel
  QR_odom_sub_ = nh_.subscribe<agvparking_msg::AgvOdom>("/qr_odom", 10, &PoseAlign::qrInfoCallback, this);
  boost::shared_ptr<boost::thread> align_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseAlign::getPoseError, this)));
  boost::shared_ptr<boost::thread> move_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseAlign::alignAndMove, this)));
}

PoseAlign::~PoseAlign()
{

}

void PoseAlign::qrInfoCallback(const agvparking_msg::AgvOdom::ConstPtr &qr_info)
{
  tag_mutex_.lock();
  //odom pose
  pose3f_[0] = qr_info->pose.position.x;
  pose3f_[1] = qr_info->pose.position.y;
  pose3f_[2] =  tf::getYaw(qr_info->pose.orientation);  //里程计航向角。
  //odom velocity.
  vel3f_[0] = qr_info->twist.linear.x;
  vel3f_[1] = qr_info->twist.linear.y;
  vel3f_[2] = qr_info->twist.angular.z;
//  if(qr_info->qr.Tagnum)   //识别到二维码。
  {
    qr_pose_.tag_num = (unsigned short int)qr_info->qr.Tagnum;
    qr_pose_.x = qr_info->qr.x/1000.0;         //m.
    qr_pose_.y = qr_info->qr.y/1000.0;         //m.
    qr_pose_.angle = angleMapAndRad(qr_info->qr.angle);        //rad.
    std::cout << "error_x=" << qr_pose_.x<< std::endl;
    std::cout << "error_y=" << qr_pose_.y<< std::endl;
    std::cout << "error_angle=" << qr_pose_.angle<< std::endl;
//    std::cout << "tag_num=" << qr_pose_.tag_num << std::endl;
  }
  tag_mutex_.unlock();
}

void PoseAlign::getPoseError()
{
  while(true)
  {
    if(tag_mutex_.try_lock())
    {
      if((fabs(qr_pose_.x) > position_tolerance_) ||
          (fabs(qr_pose_.y) > position_tolerance_) ||
          (fabs(qr_pose_.angle) > angle_tolerance_))
      {
        error_mutex_.lock();
        aligning_ = true;                       //需要调整。
 //       std::cout << "aligning_" << aligning_<<std::endl;
//        error_.x = SIGN(qr_pose_.x) * (fabs(qr_pose_.x) - position_tolerance_);       //根据正负号判断偏差方向。
//        error_.y = SIGN(qr_pose_.y) * (fabs(qr_pose_.y) - position_tolerance_);
//        error_.angle = SIGN(qr_pose_.angle) * (fabs(qr_pose_.angle) - angle_tolerance_);
//        error_.tag_num = qr_pose_.tag_num;
        if(fabs(qr_pose_.x) > position_tolerance_)
          x_aligning_ = true;
        else
          x_aligning_ = false;
        if(fabs(qr_pose_.y) > position_tolerance_)
          y_aligning_ = true;
        else
          y_aligning_ = false;
        if(fabs(qr_pose_.angle) > angle_tolerance_)
          yaw_aligning_ = true;
        else
          yaw_aligning_ = false;
        error_mutex_.unlock();
//        std::cout << "aligning_x" << x_aligning<< std::endl;
//        std::cout << "aligning_y" << y_aligning<< std::endl;
//        std::cout << "aligning_yaw" << yaw_aligning<< std::endl;
      }
      else
      {
        aligning_ = false;      //调整完毕。
        x_aligning_ = false;
        y_aligning_ = false;
        yaw_aligning_ = false;
      }
      tag_mutex_.unlock();
    }
    waitMilli(10);
  }
}
void PoseAlign::alignAndMove()
{
  float vx = 0.0, vy = 0.0, vw = 0.0;
  while(true)
  {
    waitMilli(50);
  //  std::cout << "thread" << std::endl;
//    std::cout << "aligning=" << aligning_<< std::endl;
//    std::cout << "tag_num=" << error_.tag_num<< std::endl;
    if(tag_mutex_.try_lock())
    {
//      std::cout <<"tag_num=" << qr_pose_.tag_num<< std::endl;
      if(!aligning_)    //不需要调整
      {
        if(qr_pose_.tag_num == 1)
        {
          if(motion_state_.start  && !moving_)       //起始
          {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 1.0;      //抬升。
            cmd_vel_.angular.z = 0.0;
            waitMilli(200);
            moving_ = true;
          }
          else if(moving_)
          {
            waitMilli(100);
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = INIT_VEL;
            cmd_vel_.linear.z = 1.0;      //抬升。
            cmd_vel_.angular.z = 0.0;
            //update stae.
            motion_state_.moving_dir = "Y+";
            motion_state_.start = false;
            moving_ = true;
          }
          //publish.
          cmd_vel_pub_.publish(cmd_vel_);
        }
        if(qr_pose_.tag_num == 3)
        {
          if(motion_state_.expeted_dir == "X+")
          {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.z = 0.0;
            waitMilli(200);              //原地调整完成，1秒后启动。
            cmd_vel_.linear.x = INIT_VEL;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 1.0;      //抬升。
            cmd_vel_.angular.z = 0.0;

            //update stae.
            motion_state_.moving_dir = "X+";
          }
          else if(motion_state_.expeted_dir == "X-")
          {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.z = 0.0;
            waitMilli(200);              //原地调整完成，1秒后启动。
            cmd_vel_.linear.x = -INIT_VEL;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 1.0;      //抬升。
            cmd_vel_.angular.z = 0.0;

            //update stae.
            motion_state_.moving_dir = "X-";
          }
          else if(motion_state_.expeted_dir == "Y-")
          {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.z = 0.0;
            waitMilli(200);              //原地调整完成，1秒后启动。
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = INIT_VEL;
            cmd_vel_.linear.z = 1.0;      //抬升。
            cmd_vel_.angular.z = 0.0;

            //update stae.
            motion_state_.moving_dir = "Y-";
          }
          //publish.
          cmd_vel_pub_.publish(cmd_vel_);
        }

        tag_mutex_.unlock();
        continue;
      }
      //align.
      switch (qr_pose_.tag_num) {
        case 0:
          //do nothing.
          break;
        case 1:         //align at first place.
          if(!moving_)
            alignInit(qr_pose_);

          //update move state.
        //std::cout << "aligning_1"<< std::endl;
          motion_state_.moving_dir = "";
          motion_state_.start = true;
          motion_state_.expeted_dir = "Y+";
          break;
        case 2:         //引导。
          alignAlgorithm(qr_pose_, vx, vy, vw);
          break;
        case 3:         //切换运动方向。
          //stop
      //    stop();
          motion_state_.expeted_dir = "X+";
          alignInit(qr_pose_);
          break;
        case 4:         //加/减速。
          alignAlgorithm(qr_pose_, vx, vy, vw);
          if(motion_state_.speedup)
          {
            motion_state_.speedup = false;
            if(motion_state_.expeted_dir == "Y+")
            {
              cmd_vel_.linear.x = vx;
              cmd_vel_.linear.y = TRAVEL_VEL_X;
            }
            else if(motion_state_.expeted_dir == "Y-")
            {
              cmd_vel_.linear.x = vx;
              cmd_vel_.linear.y = -TRAVEL_VEL_X;
            }
            else if(motion_state_.expeted_dir == "X+")
            {
              cmd_vel_.linear.x = TRAVEL_VEL_X;
              cmd_vel_.linear.y = vy;
            }

            else if(motion_state_.expeted_dir == "X-")
            {

              cmd_vel_.linear.x = -TRAVEL_VEL_X;
              cmd_vel_.linear.y = vy;
            }
          }
          else
          {
            if(motion_state_.expeted_dir == "Y+")
            {
               cmd_vel_.linear.x = vx;
               cmd_vel_.linear.y = INIT_VEL;
            }
            else if(motion_state_.expeted_dir == "Y-")
            {
               cmd_vel_.linear.x = vx;
               cmd_vel_.linear.y = -INIT_VEL;
            }
            else if(motion_state_.expeted_dir == "X+")
            {
               cmd_vel_.linear.x = INIT_VEL;
               cmd_vel_.linear.y = vy;
            }
            else if(motion_state_.expeted_dir == "X-")
            {
               cmd_vel_.linear.x = -INIT_VEL;
               cmd_vel_.linear.y = vy;
            }
          }
          cmd_vel_.linear.z = 1.0;
          cmd_vel_.angular.z = vw;
          break;
        case 5:
          break;
        default:

          break;
      }
      tag_mutex_.unlock();
    }
    waitMilli(50);
  }

}

void PoseAlign::alignAlgorithm(const QRPose &error, float &vx, float &vy, float &vw)
{
  unsigned short int tx = 0, ty = 0, tYaw = 0;
  if((motion_state_.moving_dir == "X+" || motion_state_.moving_dir == "X-") && y_aligning_ )   //moving at x direction.
  {
    if(motion_state_.moving_dir == "X+")
      vx = TRAVEL_VEL_X;
    else if(motion_state_.moving_dir == "X-")
      vx = -TRAVEL_VEL_X;

    vy = -SIGN(error.y) * ALIGN_VEL;    //y方向上进行微调。
    ty = (unsigned short int)(1000 * fabs(error.y) / ALIGN_VEL);     //调整时间,ms。
  }
  else if((motion_state_.moving_dir == "Y+" || motion_state_.moving_dir == "Y-") && x_aligning_)   //moving at y direction.
  {
    if(motion_state_.moving_dir == "Y+")
      vy = TRAVEL_VEL_X;
    else if(motion_state_.moving_dir == "Y-")
      vy = -TRAVEL_VEL_X;

    vx = -SIGN(error.x) * ALIGN_VEL;
    tx = (unsigned short int)(1000 * fabs(error.x) / ALIGN_VEL);     //调整时间。
  }
  if(yaw_aligning_)
  {
    vw = -SIGN(error.angle) * ALIGN_VEL;
    tYaw = (unsigned short int)(1000 * fabs(error.angle) / TRAVEL_VEL_X);
  }
  const unsigned short int align_time = std::max(std::max(tx, ty), tYaw);
  //publish.
  cmd_vel_.linear.z = 1.0;
  cmd_vel_pub_.publish(cmd_vel_);
  waitMilli(align_time);
  //reset.
  x_aligning_ = y_aligning_ = yaw_aligning_ = false;
}
void PoseAlign::alignInit(const QRPose &error)
{
  float vx = 0.0, vy = 0.0, vw = 0.0;
  if(x_aligning_)
    vx = -SIGN(error.x) * ALIGN_VEL;
  if(y_aligning_)
    vy = -SIGN(error.y) * ALIGN_VEL;
  if(yaw_aligning_)
    vw = -SIGN(error.angle) * ALIGN_VEL;
  cmd_vel_.linear.x = vx;
  cmd_vel_.linear.y = vy;
  cmd_vel_.angular.z = vw;

  //publish.
  cmd_vel_pub_.publish(cmd_vel_);
}

float PoseAlign::angleMapAndRad(const float &angle)
{
  unsigned short int angle_mapped = 0;  //[-180, 180]
  if(angle > 180)
    angle_mapped = angle - 360;
  else
    angle_mapped = angle;
  //degree to radian.
  float angle_rad = (float)angle_mapped * (M_PI/180.0);
  return angle_rad;
}

void PoseAlign::setState(const MoveState &state)
{
  state_mutex_.lock();
  motion_state_.moving_dir = state.moving_dir;
  motion_state_.start = state.start;
  motion_state_.expeted_dir = state.expeted_dir;
  state_mutex_.unlock();
}

PoseAlign::MoveState PoseAlign::getState()
{
  return motion_state_;
}

void PoseAlign::stop()
{
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.linear.z = 1.0;
  cmd_vel_.angular.z = 0.0;
  //publish
  cmd_vel_pub_.publish(cmd_vel_);
}
