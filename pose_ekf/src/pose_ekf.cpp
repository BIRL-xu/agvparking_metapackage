/*
 * pose_ekf.cpp
 *
 *  Created on: Apr 11, 2017
 *      Author: paul
 */

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "pose_ekf/pose_ekf.h"

#include <math.h>

const float QRdist = 0.5;     //QR码的间距，默认50cm.
const float Theta1 = 0.005;      //标定.
const float Theta3 = -0.005;
const tf::Vector3 C2R(-1.21775, 0.243, 0.0);      //标定. 机器人在相机坐标系下．
const tf::Vector3 R2C(1.21775, -0.243, 0.0);      //　相机在机器人坐标系下．

PoseEKF::PoseEKF():nh_(ros::NodeHandle("~")),initialized_(false), QRcoor_(0.0, 0.0, 0.0), factor_(1.0),error_buffer_size_(10)
{

  ref_.setZero();
  setTransMatrix();
  nh_.param("debug", debug_, false);
  current_time_ = last_time_ = ros::Time::now();

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 5, &PoseEKF::odomCallback, this);
  QR_sub_ = nh_.subscribe<agvparking_msg::AgvOdom>("/qr_odom", 1, &PoseEKF::QRCallback, this);
  tf::Matrix3x3 err_que(0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0);
  for(int i = 0; i< error_buffer_size_; i++)
  {
    error_buffer_.push(err_que);
  }

//  boost::shared_ptr<boost::thread> ekf_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseEKF::UKFupdate, this)));
 boost::shared_ptr<boost::thread> ukf_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseEKF::UKFUpdateOdom, this)));
//boost::shared_ptr<boost::thread> aukf_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseEKF::AUKFUpdataOdom, this)));
}
PoseEKF::~PoseEKF()
{
  if(out_ekf_.is_open())
    out_ekf_.close();
}
void PoseEKF::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  boost::unique_lock<boost::timed_mutex> lock(QRmutex_);
  odom_ = *msg;
  ROS_DEBUG("Receive odometry's x component is (%.3f)", odom_.pose.pose.position.x);
}
void PoseEKF::QRCallback(const agvparking_msg::AgvOdom::ConstPtr &QR_info)
{
  boost::unique_lock<boost::timed_mutex> lock(QRmutex_);
  //header
  QRinfo_.header.stamp = QR_info->header.stamp;
  QRinfo_.header.frame_id = QR_info->header.frame_id;
  QRinfo_.child_frame_id = QR_info->child_frame_id;
  //position
  QRinfo_.pose.position = QR_info->pose.position;
  //pose
  QRinfo_.pose.orientation = QR_info->pose.orientation;
  //vel
  QRinfo_.twist = QR_info->twist;

  //QR
  QRinfo_.QRtag.Tagnum = QR_info->QRtag.Tagnum;
  QRinfo_.QRtag.x = QR_info->QRtag.x;   //m
  QRinfo_.QRtag.y = QR_info->QRtag.y;

//  QRinfo_.QRtag.x = 0.01;   //m
//  QRinfo_.QRtag.y = 0.005;
  QRinfo_.QRtag.angle = QR_info->QRtag.angle;
  QRinfo_.QRtag.covariance = QR_info->QRtag.covariance;

  boost::timed_mutex *ptr = lock.release();
  ptr->unlock();
}

void PoseEKF::update()
{
  while(true)
  {
    waitMilli(10);
 //   ROS_INFO("debug = %s", debug_?"true":"false");
    agvparking_msg::AgvOdom QRdata;
    QRdata.pose.orientation.z = 1.0;

    static tf::Vector3 vel;      //控制量u.
   // vel.setZero();        //清零.
    boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
    if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(10)))
    {
      QRdata = QRinfo_;
      ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
      lock.unlock();
      vel.setX(QRdata.twist.linear.x);
      vel.setY(QRdata.twist.linear.y);
      vel.setZ(QRdata.twist.angular.z);
    }
    else
    {
      ROS_WARN("Copy QR's data failed!");
      continue;
    }
    if(QRdata.QRtag.Tagnum)     //进行融合.
    {
      if(!initialized_)
      {
        if(debug_)
        {
          out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ekf_pose.txt");
          if(!out_ekf_.is_open())
            ROS_WARN("'ekf_pose.txt' open failed.");
        }
        t_last_ = QRdata.header.stamp.toSec();
        prev_state_.P.setValue(0.01, 0.0, 0.0,
                               0.0, 0.01, 0.0,
                               0.0, 0.0, 0.01);
        measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

        Q_.setValue(0.0005, 0.0, 0.0,
                    0.0, 0.0005, 0.0,
                    0.0, 0.0, 0.0005);
/*        R_.setValue(QRdata.QRtag.covariance[0], QRdata.QRtag.covariance[1], QRdata.QRtag.covariance[2],
                    QRdata.QRtag.covariance[3], QRdata.QRtag.covariance[4], QRdata.QRtag.covariance[5],
                    QRdata.QRtag.covariance[6], QRdata.QRtag.covariance[7], QRdata.QRtag.covariance[8]);
*/
        R_.setValue(0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.01);
        initialized_ = true;
        continue;
      }
      const float dT = fabs(QRdata.header.stamp.toSec() - t_last_);
   //   const float dT = 0.056;
      //ROS_DEBUG("dt=%.4f", dT);
    //  ROS_INFO("dt = %.3f", dT);
      if(dT < 0)
      {
        ROS_ERROR("we can't estimate past state.");
        continue;
      }

      //四舍五入
      int pre_pose_x = 0, pre_pose_y = 0;
      if(QRdata.pose.position.x >= 0)
      {
        pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 + 5)/10.0);
      }
      else
        pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 - 5)/10.0);
      if(QRdata.pose.position.y >= 0)
      {
        pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 + 5)/10.0);
      }
      else
        pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 - 5)/10.0);

      const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
      const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
 //     const tf::Vector3 dQR(QRcoor_curr);    //QR1->QR2.
 //     ROS_INFO("QR coordinate=(%.3f, %.3f, %.3f)", QRcoor_curr.x(), QRcoor_curr.y(), QRcoor_curr.z());

//      if(fabs(QRcoor_.distance(QRcoor_curr)) >= (0.5*QRdist) || QRcoor_curr.x() == 0.0)
//      {
        QRcoor_ = QRcoor_curr;         //更新.

      //1.time update process.



/*
      prev_state_.X.setValue(QRdata.pose.position.x-post_state_.X.x(),                    //(r2-r1):x
                             QRdata.pose.position.y-post_state_.X.y(),                  //(r2-r1):y
                             tf::getYaw(QRdata.pose.orientation)-post_state_.X.z());    //(r2-r1):yaw //先验估计.
*/

        //前一估值点的选取很关键.

        prev_state_.X.setValue(QRdata.pose.position.x-ref_.x(),                    //(r2-r1):x
                               QRdata.pose.position.y-ref_.y(),                  //(r2-r1):y
                               tf::getYaw(QRdata.pose.orientation)-ref_.z());    //(r2-r1):yaw //先验估计.

/*
        prev_state_.X.setValue(QRdata.pose.position.x,                    //(r2-r1):x
                               QRdata.pose.position.y,                  //(r2-r1):y
                               tf::getYaw(QRdata.pose.orientation));    //(r2-r1):yaw //先验估计.
*/
/*      ROS_INFO("odom:(%.3f, %.3f)",QRdata.pose.position.x, QRdata.pose.position.y);
      ROS_INFO("ref:(%.3f, %.3f, %.3f)",ref_.x(), ref_.y(), ref_.z());
      ROS_INFO("QR1->QR2:(%.3f, %.3f, %.3f)",dQR.x(), dQR.y(), dQR.z());
      ROS_INFO("prev_x:(%.3f, %.3f, %.3f)",prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());*/

      ref_.setValue(QRdata.pose.position.x, QRdata.pose.position.y, tf::getYaw(QRdata.pose.orientation));

      ROS_DEBUG("prev_x:(%.3f, %.3f, %.3f)",prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
//      if(dQR.x() == 1.5 || dQR.x() == 1.0)

      //EKF.
      //G
      const float Theta_G = post_state_.X.z()+vel.z()*dT;                   //G矩阵的theta.
      const float g11 = 1.0, g12 = 0.0, g13 = -(vel.x()*sin(Theta_G) + vel.y()*cos(Theta_G))*dT;
      const float g21 = 0.0, g22 = 1.0, g23 = (vel.x()*cos(Theta_G) - vel.y()*sin(Theta_G))*dT;
      const float g31 = 0.0, g32 = 0.0, g33 = 1.0;
      const tf::Matrix3x3 G(g11, g12, g13,
                            g21, g22, g23,
                            g31, g32, g33);
      //H
      const float Theta_H = Theta1 + prev_state_.X.z() + Theta3 + measu_last_.z();          //H矩阵的theta.
      const float param1 = measu_last_.x()*cos(Theta3) - measu_last_.y()*sin(Theta3) + R2C.x();
      const float param2 = measu_last_.x()*sin(Theta3) + measu_last_.y()*cos(Theta3) + R2C.y();
      const float h11 = cos(Theta1), h12 = -sin(Theta1);
      const float h21 = sin(Theta1), h22 = cos(Theta1);
      const float h31 = 0.0, h32 = 0.0;
      const float h13 = -dQR.x()*sin(Theta_H)-dQR.y()*cos(Theta_H)-param1*sin(Theta1+prev_state_.X.z())-param2*cos(Theta1+prev_state_.X.z());
      const float h23 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+param1*cos(Theta1+prev_state_.X.z())-param2*sin(Theta1+prev_state_.X.z());
      const float h33 = 1.0;
      const tf::Matrix3x3 H(h11, h12, h13,
                            h21, h22, h23,
                            h31, h32, h33);
      //P_prev
      prev_state_.P = addMatrix(G*post_state_.P*G.transpose(), Q_);

      //2.measurement update process.
      //当前观测量
      tf::Vector3 measu_curr(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
      const float z1 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+
                        param1*cos(Theta1+prev_state_.X.z())-
                        param2*sin(Theta1+prev_state_.X.z())+
                        prev_state_.X.x()*cos(Theta1)-prev_state_.X.y()*sin(Theta1)+
                        C2R.x();
      const float z2 = dQR.x()*sin(Theta_H)+dQR.y()*cos(Theta_H)+
                        param1*sin(Theta1+prev_state_.X.z())+
                        param2*cos(Theta1+prev_state_.X.z())+
                        prev_state_.X.x()*sin(Theta1)+prev_state_.X.y()*cos(Theta1)+
                        C2R.y();
      const float z3 = Theta_H;
      const tf::Vector3 z_prev(z1, z2, z3);     //预估观测量.

      //K
      const tf::Matrix3x3 K = prev_state_.P*H.transpose()*(addMatrix(H*prev_state_.P*H.transpose(), R_).inverse());
      post_state_.X += (prev_state_.X + K*(measu_curr - z_prev));

      post_state_.P = reduceMatrix(tf::Matrix3x3::getIdentity(), K*H)*prev_state_.P;

      //迭代更新
      {
        for(int i = 0; i<5; i++){
        prev_state_.X.setValue(post_state_.X.x()-ref_.x(), post_state_.X.y()-ref_.y(), post_state_.X.z()-ref_.z());
        const float Theta_H = Theta1 + prev_state_.X.z() + Theta3 + measu_last_.z();          //H矩阵的theta.
        const float param1 = measu_last_.x()*cos(Theta3) - measu_last_.y()*sin(Theta3) + R2C.x();
        const float param2 = measu_last_.x()*sin(Theta3) + measu_last_.y()*cos(Theta3) + R2C.y();
        const float h11 = cos(Theta1), h12 = -sin(Theta1);
        const float h21 = sin(Theta1), h22 = cos(Theta1);
        const float h31 = 0.0, h32 = 0.0;
        const float h13 = -dQR.x()*sin(Theta_H)-dQR.y()*cos(Theta_H)-param1*sin(Theta1+prev_state_.X.z())-param2*cos(Theta1+prev_state_.X.z());
        const float h23 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+param1*cos(Theta1+prev_state_.X.z())-param2*sin(Theta1+prev_state_.X.z());
        const float h33 = 1.0;
        const tf::Matrix3x3 H(h11, h12, h13,
                              h21, h22, h23,
                              h31, h32, h33);
        //P_prev
       // prev_state_.P = addMatrix(G*post_state_.P*G.transpose(), Q_);
        prev_state_.P = post_state_.P;
        //2.measurement update process.
        //当前观测量
        tf::Vector3 measu_curr(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
        const float z1 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+
                          param1*cos(Theta1+prev_state_.X.z())-
                          param2*sin(Theta1+prev_state_.X.z())+
                          prev_state_.X.x()*cos(Theta1)-prev_state_.X.y()*sin(Theta1)+
                          C2R.x();
        const float z2 = dQR.x()*sin(Theta_H)+dQR.y()*cos(Theta_H)+
                          param1*sin(Theta1+prev_state_.X.z())+
                          param2*cos(Theta1+prev_state_.X.z())+
                          prev_state_.X.x()*sin(Theta1)+prev_state_.X.y()*cos(Theta1)+
                          C2R.y();
        const float z3 = Theta_H;
        const tf::Vector3 z_prev(z1, z2, z3);     //预估观测量.

        //K
        const tf::Matrix3x3 K = prev_state_.P*H.transpose()*(addMatrix(H*prev_state_.P*H.transpose(), R_).inverse());
        post_state_.X += (prev_state_.X + K*(measu_curr - z_prev));

        post_state_.P = reduceMatrix(tf::Matrix3x3::getIdentity(), K*H)*prev_state_.P;

        //sigma points.
        }

      }

/*      if(QRdata.QRtag.Tagnum == 4)
      {
//        ROS_INFO_ONCE("Tag_num:%d", QRdata.QRtag.Tagnum);
//        ROS_INFO_ONCE("QR coordinate:(%.3f, %.3f, %.3f)",QRcoor_curr.x(), QRcoor_curr.y(), QRcoor_curr.z());

        ROS_INFO_ONCE("odom:(%.3f, %.3f)",QRdata.pose.position.x, QRdata.pose.position.y);
        ROS_INFO_ONCE("prev_x:(%.3f, %.3f, %.3f)",prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
        ROS_INFO_ONCE("new_prev= %.4f", (prev_state_.X + K*(measu_curr - z_prev)).x());         //修正量太小.
      }*/

//      ROS_INFO("new_prev= %.4f", (prev_state_.X + K*(measu_curr - z_prev)).x());         //修正量太小.
      ROS_INFO("********************");
      if(debug_)
        out_ekf_ << post_state_.X.x() << ","
               << post_state_.X.y() << ","
               << post_state_.X.z() << ","
               << K.getRow(0).x() << ","
               << K.getRow(1).y() << ","
               << K.getRow(2).z() << ","
               << post_state_.P.getRow(0).x() << ","
               << post_state_.P.getRow(1).y() << ","
               << post_state_.P.getRow(2).z() << ","
               << measu_curr.x() << ","
               << measu_curr.y()
               <<std::endl;

      measu_last_ = measu_curr;
      t_last_ = QRdata.header.stamp.toSec();

//      }
    }
    else
    {
/*      post_state_.X.setX(QRdata.pose.position.x);
      post_state_.X.setY(QRdata.pose.position.y);
      post_state_.X.setZ(QRdata.pose.position.z);*/

      continue;
    }

  }
}

void PoseEKF::EKFUpdateOdom()
{
  while(true)
  {

 //   ROS_INFO("debug = %s", debug_?"true":"false");

    agvparking_msg::AgvOdom QRdata;
    QRdata.pose.orientation.z = 1.0;

    static tf::Vector3 vel;      //控制量u.
    vel.setZero();        //清零.
    boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
    if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(10)))
    {
      QRdata = QRinfo_;
      ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
      lock.unlock();
      vel.setX(QRdata.twist.linear.x);
      vel.setY(QRdata.twist.linear.y);
      vel.setZ(QRdata.twist.angular.z);
    }
    else
    {
      ROS_WARN("Copy QR's data failed!");
      continue;
    }

    //odometry.
    current_time_ = ros::Time::now();
    const float dt = (current_time_ - last_time_).toSec();

    const float dx = vel.x() * dt;
    const float dy = vel.y() * dt;
    const float dtheta = vel.z() * dt;
    //odometry updates the pose.
    double temp_x, temp_y, temp_z;
    temp_x = ekf_pose_.x() + dx * cos(ekf_pose_.z() + dtheta/2.0) - dy * sin(ekf_pose_.z() + dtheta/2.0);
    temp_y = ekf_pose_.y() + dx * sin(ekf_pose_.z() + dtheta/2.0) + dy * cos(ekf_pose_.z() + dtheta/2.0);
    temp_z = ekf_pose_.z() + dtheta;

    ekf_pose_.setX(temp_x);
    ekf_pose_.setY(temp_y);
    ekf_pose_.setZ(temp_z);

    if(ekf_pose_.z() > M_PI)
      ekf_pose_.setZ(ekf_pose_.z() - 2*M_PI);
    else if(ekf_pose_.z() < -M_PI)
      ekf_pose_.setZ(ekf_pose_.z() + 2*M_PI);

 //   ROS_INFO("pose:(%.3f, %.3f, %.3f)",pose_.x(), pose_.y(), pose_.z());
    if(QRdata.QRtag.Tagnum)     //进行融合.
    {
      if(!initialized_)
      {
        if(debug_)
        {
          out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ekf_pose.txt");
          if(!out_ekf_.is_open())
            ROS_WARN("'ekf_pose.txt' open failed.");
        }
 //       t_last_ = QRdata.header.stamp.toSec();
        prev_state_.P.setValue(0.01, 0.0, 0.0,
                               0.0, 0.01, 0.0,
                               0.0, 0.0, 0.01);
        measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

        Q_.setValue(0.05, 0.0, 0.0,
                    0.0, 0.05, 0.0,
                    0.0, 0.0, 0.05);
/*        R_.setValue(QRdata.QRtag.covariance[0], QRdata.QRtag.covariance[1], QRdata.QRtag.covariance[2],
                    QRdata.QRtag.covariance[3], QRdata.QRtag.covariance[4], QRdata.QRtag.covariance[5],
                    QRdata.QRtag.covariance[6], QRdata.QRtag.covariance[7], QRdata.QRtag.covariance[8]);
*/
        R_.setValue(0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.01);
        initialized_ = true;
        continue;
      }
 //     const float dT = fabs(QRdata.header.stamp.toSec() - t_last_);
   //   const float dT = 0.056;
      //ROS_DEBUG("dt=%.4f", dT);
    //  ROS_INFO("dt = %.3f", dT);
//      if(dT < 0)
//      {
//        ROS_ERROR("we can't estimate past state.");
//        continue;
//      }

      //四舍五入
      int pre_pose_x = 0, pre_pose_y = 0;
      if(ekf_pose_.x() >= 0)
      {
        pre_pose_x = (int)((ekf_pose_.x()/QRdist*10 + 5)/10.0);
      }
      else
        pre_pose_x = (int)((ekf_pose_.x()/QRdist*10 - 5)/10.0);
      if(ekf_pose_.y() >= 0)
      {
        pre_pose_y = (int)((ekf_pose_.y()/QRdist*10 + 5)/10.0);
      }
      else
        pre_pose_y = (int)((ekf_pose_.y()/QRdist*10 - 5)/10.0);

      const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
      const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
 //     const tf::Vector3 dQR(QRcoor_curr);    //QR1->QR2.
 //     ROS_INFO("QR coordinate=(%.3f, %.3f, %.3f)", QRcoor_curr.x(), QRcoor_curr.y(), QRcoor_curr.z());

//      if(fabs(QRcoor_.distance(QRcoor_curr)) >= (0.5*QRdist) || QRcoor_curr.x() == 0.0)
//      {
        QRcoor_ = QRcoor_curr;         //更新.

      //1.time update process.



/*
      prev_state_.X.setValue(QRdata.pose.position.x-post_state_.X.x(),                    //(r2-r1):x
                             QRdata.pose.position.y-post_state_.X.y(),                  //(r2-r1):y
                             tf::getYaw(QRdata.pose.orientation)-post_state_.X.z());    //(r2-r1):yaw //先验估计.
*/

        //前一估值点的选取很关键.

        prev_state_.X.setValue(ekf_pose_.x()-ref_.x(),                    //(r2-r1):x
                               ekf_pose_.y()-ref_.y(),                  //(r2-r1):y
                               ekf_pose_.z()-ref_.z());    //(r2-r1):yaw //先验估计.

/*
        prev_state_.X.setValue(QRdata.pose.position.x,                    //(r2-r1):x
                               QRdata.pose.position.y,                  //(r2-r1):y
                               tf::getYaw(QRdata.pose.orientation));    //(r2-r1):yaw //先验估计.
*/
/*      ROS_INFO("odom:(%.3f, %.3f)",QRdata.pose.position.x, QRdata.pose.position.y);
      ROS_INFO("ref:(%.3f, %.3f, %.3f)",ref_.x(), ref_.y(), ref_.z());
      ROS_INFO("QR1->QR2:(%.3f, %.3f, %.3f)",dQR.x(), dQR.y(), dQR.z());
      ROS_INFO("prev_x:(%.3f, %.3f, %.3f)",prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());*/

      ref_.setValue(QRdata.pose.position.x, QRdata.pose.position.y, tf::getYaw(QRdata.pose.orientation));
//      ref_.setValue(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());

      ROS_INFO("pose:(%.3f, %.3f, %.3f)",ekf_pose_.x(), ekf_pose_.y(), ekf_pose_.z());

      ROS_INFO("post_state:(%.3f, %.3f, %.3f)",post_state_.X.x(), post_state_.X.y(), post_state_.X.z());

      ROS_INFO("prev_state:(%.3f, %.3f, %.3f)",prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
//      if(dQR.x() == 1.5 || dQR.x() == 1.0)

      //EKF.
      //G
      const float Theta_G = post_state_.X.z()+vel.z()*dt;                   //G矩阵的theta.
      const float g11 = 1.0, g12 = 0.0, g13 = -(vel.x()*sin(Theta_G) + vel.y()*cos(Theta_G))*dt;
      const float g21 = 0.0, g22 = 1.0, g23 = (vel.x()*cos(Theta_G) - vel.y()*sin(Theta_G))*dt;
      const float g31 = 0.0, g32 = 0.0, g33 = 1.0;
      const tf::Matrix3x3 G(g11, g12, g13,
                            g21, g22, g23,
                            g31, g32, g33);
      //H
      const float Theta_H = Theta1 + prev_state_.X.z() + Theta3 + measu_last_.z();          //H矩阵的theta.
      const float param1 = measu_last_.x()*cos(Theta3) - measu_last_.y()*sin(Theta3) + R2C.x();
      const float param2 = measu_last_.x()*sin(Theta3) + measu_last_.y()*cos(Theta3) + R2C.y();
      const float h11 = cos(Theta1), h12 = -sin(Theta1);
      const float h21 = sin(Theta1), h22 = cos(Theta1);
      const float h31 = 0.0, h32 = 0.0;
      const float h13 = -dQR.x()*sin(Theta_H)-dQR.y()*cos(Theta_H)-param1*sin(Theta1+prev_state_.X.z())-param2*cos(Theta1+prev_state_.X.z());
      const float h23 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+param1*cos(Theta1+prev_state_.X.z())-param2*sin(Theta1+prev_state_.X.z());
      const float h33 = 1.0;
      const tf::Matrix3x3 H(h11, h12, h13,
                            h21, h22, h23,
                            h31, h32, h33);
      //P_prev
      prev_state_.P = addMatrix(G*post_state_.P*G.transpose(), Q_);

      //2.measurement update process.
      //当前观测量
      tf::Vector3 measu_curr(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
      const float z1 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+
                        param1*cos(Theta1+prev_state_.X.z())-
                        param2*sin(Theta1+prev_state_.X.z())+
                        prev_state_.X.x()*cos(Theta1)-prev_state_.X.y()*sin(Theta1)+
                        C2R.x();
      const float z2 = dQR.x()*sin(Theta_H)+dQR.y()*cos(Theta_H)+
                        param1*sin(Theta1+prev_state_.X.z())+
                        param2*cos(Theta1+prev_state_.X.z())+
                        prev_state_.X.x()*sin(Theta1)+prev_state_.X.y()*cos(Theta1)+
                        C2R.y();
      const float z3 = Theta_H;
      const tf::Vector3 z_prev(z1, z2, z3);     //预估观测量.

      //K
      const tf::Matrix3x3 K = prev_state_.P*H.transpose()*(addMatrix(H*prev_state_.P*H.transpose(), R_).inverse());
      post_state_.X += (prev_state_.X + K*(measu_curr - z_prev));

      post_state_.P = reduceMatrix(tf::Matrix3x3::getIdentity(), K*H)*prev_state_.P;

      //迭代更新
      {
  //      for(int i = 0; i<5; i++){
  //      prev_state_.X.setValue(post_state_.X.x()-ref_.x(), post_state_.X.y()-ref_.y(), post_state_.X.z()-ref_.z());
        const float Theta_H = Theta1 + prev_state_.X.z() + Theta3 + measu_last_.z();          //H矩阵的theta.
        const float param1 = measu_last_.x()*cos(Theta3) - measu_last_.y()*sin(Theta3) + R2C.x();
        const float param2 = measu_last_.x()*sin(Theta3) + measu_last_.y()*cos(Theta3) + R2C.y();
        const float h11 = cos(Theta1), h12 = -sin(Theta1);
        const float h21 = sin(Theta1), h22 = cos(Theta1);
        const float h31 = 0.0, h32 = 0.0;
        const float h13 = -dQR.x()*sin(Theta_H)-dQR.y()*cos(Theta_H)-param1*sin(Theta1+prev_state_.X.z())-param2*cos(Theta1+prev_state_.X.z());
        const float h23 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+param1*cos(Theta1+prev_state_.X.z())-param2*sin(Theta1+prev_state_.X.z());
        const float h33 = 1.0;
        const tf::Matrix3x3 H(h11, h12, h13,
                              h21, h22, h23,
                              h31, h32, h33);
        //P_prev
       // prev_state_.P = addMatrix(G*post_state_.P*G.transpose(), Q_);
        prev_state_.P = post_state_.P;
        //2.measurement update process.
        //当前观测量
        tf::Vector3 measu_curr(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
        const float z1 = dQR.x()*cos(Theta_H)-dQR.y()*sin(Theta_H)+
                          param1*cos(Theta1+prev_state_.X.z())-
                          param2*sin(Theta1+prev_state_.X.z())+
                          prev_state_.X.x()*cos(Theta1)-prev_state_.X.y()*sin(Theta1)+
                          C2R.x();
        const float z2 = dQR.x()*sin(Theta_H)+dQR.y()*cos(Theta_H)+
                          param1*sin(Theta1+prev_state_.X.z())+
                          param2*cos(Theta1+prev_state_.X.z())+
                          prev_state_.X.x()*sin(Theta1)+prev_state_.X.y()*cos(Theta1)+
                          C2R.y();
        const float z3 = Theta_H;
        const tf::Vector3 z_prev(z1, z2, z3);     //预估观测量.

        //K
        const tf::Matrix3x3 K = prev_state_.P*H.transpose()*(addMatrix(H*prev_state_.P*H.transpose(), R_).inverse());
        post_state_.X += (prev_state_.X + K*(measu_curr - z_prev));

        post_state_.P = reduceMatrix(tf::Matrix3x3::getIdentity(), K*H)*prev_state_.P;
        ekf_pose_.setValue(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
 //       }

      }

/*      if(QRdata.QRtag.Tagnum == 4)
      {
//        ROS_INFO_ONCE("Tag_num:%d", QRdata.QRtag.Tagnum);
//        ROS_INFO_ONCE("QR coordinate:(%.3f, %.3f, %.3f)",QRcoor_curr.x(), QRcoor_curr.y(), QRcoor_curr.z());

        ROS_INFO_ONCE("odom:(%.3f, %.3f)",QRdata.pose.position.x, QRdata.pose.position.y);
        ROS_INFO_ONCE("prev_x:(%.3f, %.3f, %.3f)",prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
        ROS_INFO_ONCE("new_prev= %.4f", (prev_state_.X + K*(measu_curr - z_prev)).x());         //修正量太小.
      }*/

//      ROS_INFO("new_prev= %.4f", (prev_state_.X + K*(measu_curr - z_prev)).x());         //修正量太小.
      ROS_INFO("********************");
      if(debug_)
        out_ekf_ << post_state_.X.x() << ","
               << post_state_.X.y() << ","
               << post_state_.X.z() << ","
               << K.getRow(0).x() << ","
               << K.getRow(1).y() << ","
               << K.getRow(2).z() << ","
               << post_state_.P.getRow(0).x() << ","
               << post_state_.P.getRow(1).y() << ","
               << post_state_.P.getRow(2).z() << ","
               << measu_curr.x() << ","
               << measu_curr.y()
               <<std::endl;

      measu_last_ = measu_curr;
      t_last_ = QRdata.header.stamp.toSec();

//      }
    }
    else
    {
/*      post_state_.X.setX(QRdata.pose.position.x);
      post_state_.X.setY(QRdata.pose.position.y);
      post_state_.X.setZ(QRdata.pose.position.z);*/

 //     continue;
    }
    waitMilli(80);
    last_time_ = current_time_;     //update last time.
  }
}

void PoseEKF::UKFupdate()
{
  while(true)
  {
    bool update = false;
    waitMilli(20);
 //   ROS_INFO("debug = %s", debug_?"true":"false");
    agvparking_msg::AgvOdom QRdata;
    QRdata.pose.orientation.x = 0.0;
    QRdata.pose.orientation.y = 0.0;
    QRdata.pose.orientation.z = 0.0;
    QRdata.pose.orientation.w = 1.0;
    const unsigned int n = 3;
    static tf::Vector3 vel;      //控制量u.
   // vel.setZero();        //清零.
    tf::Matrix3x3 K;
    tf::Vector3 measu_curr;
    measu_curr.setZero();
    boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
    if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(30)))
    {
      QRdata = QRinfo_;
      ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
    //  ROS_INFO("Get QR's data");
      lock.unlock();

      vel.setX(fabs(QRdata.twist.linear.x)<0.001?0.0:QRdata.twist.linear.x);
      vel.setY(fabs(QRdata.twist.linear.y)<0.001?0.0:QRdata.twist.linear.y);
      vel.setZ(fabs(QRdata.twist.angular.z)<0.001?0.0:QRdata.twist.angular.z);

    }
    else
    {
      ROS_WARN("Copy QR's data failed!");
      continue;
    }

    //四舍五入
    //QR coordinates.
    int pre_pose_x = 0, pre_pose_y = 0;
    if(QRdata.pose.position.x >= 0)
    {
      pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 + 5)/10.0);
    }
    else
      pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 - 5)/10.0);
    if(QRdata.pose.position.y >= 0)
    {
      pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 + 5)/10.0);
    }
    else
      pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 - 5)/10.0);
    const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
    const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
    ROS_DEBUG("QRcoor_curr=(%.2f, %.2f, %.2f)", QRcoor_curr.x(), QRcoor_curr.y(), QRcoor_curr.z());
    //raw odom pose
    const tf::Vector3 odom(QRdata.pose.position.x, QRdata.pose.position.y, angleTrunc(tf::getYaw(QRdata.pose.orientation)));
    raw_odom_.setValue(odom.x(), odom.y(),odom.z());
    ROS_DEBUG("ODOM=(%.3f, %.3f, %.3f)", odom.x(), odom.y(), odom.z());
    float forward_dist = sqrt((odom.x() - post_state_.X.x())*(odom.x() - post_state_.X.x()) + (odom.y() - post_state_.X.y())*(odom.y() - post_state_.X.y()));
    float robot_vel = sqrt(vel.x()*vel.x() + vel.y()*vel.y());

    float dT = 0.0;
    if(QRdata.QRtag.Tagnum)    //
    {
      update = true;
      if(fabs(robot_vel) < 1e-4)
        dT = 0.0;
      else
        dT = forward_dist / robot_vel;
    }
    else
      update = false;

    //UKF initialization.
    if(!initialized_)
    {
      if(debug_)
      {
        out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_pose.txt");
        if(!out_ekf_.is_open())
          ROS_ERROR("'ukf_pose.txt' open failed.");
        out_pose_error_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_pose_error.txt");
        if(!out_pose_error_.is_open())
          ROS_ERROR("'out_pose_error.txt' open failed.");
      }
      t_last_ = QRdata.header.stamp.toSec();
      prev_state_.P.setValue(0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0);
      post_state_.P.setValue(0.001, 0.0, 0.0,
                             0.0, 0.001, 0.0,
                             0.0, 0.0, 0.001);
      post_state_.X.setValue(QRdata.QRtag.x, QRdata.QRtag.y, angleTrunc(QRdata.QRtag.angle));       //The first measurement as the posterior.
      measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, angleTrunc(QRdata.QRtag.angle));       //第一次的观测量.

      Q_.setValue(0.0005, 0.0, 0.0,
                  0.0, 0.0005, 0.0,
                  0.0, 0.0, 0.0005);
      R_.setValue(0.00001, 0.0, 0.0,
                  0.0, 0.00001, 0.0,
                  0.0, 0.0, 0.00001);
      initialized_ = true;
      continue;
    }

    if(update)     //进行融合.
    {

      QRcoor_ = QRcoor_curr;         //更新.

      ref_.setValue(QRdata.pose.position.x, QRdata.pose.position.y, angleTrunc(tf::getYaw(QRdata.pose.orientation)));
      //UKF
      //1. 权重系数初始化.
      static const float alpha = 0.2;
      static const int kap = 0;
  //    const float kap = factor_;
      static const int beta = 2;
      static const float lamda = (float)(alpha*alpha*(n+kap) - n);      //n=3.
      static const float nc = (float)(n + lamda);
      //均值权重系数
      static const float wm0 = lamda/nc;
      static const tf::Vector3 wm1(1/(2*nc), 1/(2*nc), 1/(2*nc));
      static const tf::Vector3 wm2(1/(2*nc), 1/(2*nc), 1/(2*nc));
      //方差权重系数
      static const float wc0 = wm0 + (1-alpha*alpha+beta);
      static const tf::Vector3 wc1(wm1.x(), wm1.y(), wm1.z());
      static const tf::Vector3 wc2(wm2.x(), wm2.y(), wm2.z());
      static const float n_root = sqrt(nc);
      //2.post_state构造Sigma点
      //sigma(n;k-1) = post_x(k-1)+root((n+gama)P);
      tf::Vector3 post_sigma0(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
      tf::Vector3 post_sigma1((post_state_.X + n_root*rootMatrix(post_state_.P, 0)).x(),
                              (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).y(),
                              (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).z());
      tf::Vector3 post_sigma2((post_state_.X + n_root*rootMatrix(post_state_.P, 1)).x(),
                              (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).y(),
                              (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).z());
      tf::Vector3 post_sigma3((post_state_.X + n_root*rootMatrix(post_state_.P, 2)).x(),
                              (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).y(),
                              (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).z());
      tf::Vector3 post_sigma4((post_state_.X - n_root*rootMatrix(post_state_.P, 0)).x(),
                              (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).y(),
                              (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).z());
      tf::Vector3 post_sigma5((post_state_.X - n_root*rootMatrix(post_state_.P, 1)).x(),
                              (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).y(),
                              (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).z());
      tf::Vector3 post_sigma6((post_state_.X - n_root*rootMatrix(post_state_.P, 2)).x(),
                              (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).y(),
                              (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).z());
      ROS_DEBUG("******");
      ROS_DEBUG("post_P=(%.3f, %.3f, %.3f)", post_state_.P[0].x(),post_state_.P[0].y(),post_state_.P[0].z());
      ROS_DEBUG("post_P=(%.3f, %.3f, %.3f)", post_state_.P[1].x(),post_state_.P[1].y(),post_state_.P[1].z());
      ROS_DEBUG("post_P=(%.3f, %.3f, %.3f)", post_state_.P[2].x(),post_state_.P[2].y(),post_state_.P[2].z());

      ROS_DEBUG("******");
      ROS_DEBUG("root_P0=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 0).x(),
                                                 rootMatrix(post_state_.P, 0).y(),
                                                 rootMatrix(post_state_.P, 0).z());

      ROS_DEBUG("root_P1=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 1).x(),
                                                 rootMatrix(post_state_.P, 1).y(),
                                                 rootMatrix(post_state_.P, 1).z());
      ROS_DEBUG("root_P2=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 2).x(),
                                                 rootMatrix(post_state_.P, 2).y(),
                                                 rootMatrix(post_state_.P, 2).z());
      ROS_DEBUG("******");
      ROS_DEBUG("post_sigma0=(%.3f, %.3f, %.3f)", post_sigma0.x(), post_sigma0.y(), post_sigma0.z());
      ROS_DEBUG("post_sigma1=(%.3f, %.3f, %.3f)", post_sigma1.x(), post_sigma1.y(), post_sigma1.z());
      ROS_DEBUG("post_sigma2=(%.3f, %.3f, %.3f)", post_sigma2.x(), post_sigma2.y(), post_sigma2.z());
      ROS_DEBUG("post_sigma3=(%.3f, %.3f, %.3f)", post_sigma3.x(), post_sigma3.y(), post_sigma3.z());
      ROS_DEBUG("post_sigma4=(%.3f, %.3f, %.3f)", post_sigma4.x(), post_sigma4.y(), post_sigma4.z());
      ROS_DEBUG("post_sigma5=(%.3f, %.3f, %.3f)", post_sigma5.x(), post_sigma5.y(), post_sigma5.z());
      ROS_DEBUG("post_sigma6=(%.3f, %.3f, %.3f)", post_sigma6.x(), post_sigma6.y(), post_sigma6.z());

      //3.时间更新
      ////3.1 状态更新
      tf::Vector3 y0(systemFunc(post_sigma0, vel, dT).x(), systemFunc(post_sigma0, vel, dT).y(), systemFunc(post_sigma0, vel, dT).z());
      tf::Vector3 y1(systemFunc(post_sigma1, vel, dT).x(), systemFunc(post_sigma1, vel, dT).y(), systemFunc(post_sigma1, vel, dT).z());
      tf::Vector3 y2(systemFunc(post_sigma2, vel, dT).x(), systemFunc(post_sigma2, vel, dT).y(), systemFunc(post_sigma2, vel, dT).z());
      tf::Vector3 y3(systemFunc(post_sigma3, vel, dT).x(), systemFunc(post_sigma3, vel, dT).y(), systemFunc(post_sigma3, vel, dT).z());
      tf::Vector3 y4(systemFunc(post_sigma4, vel, dT).x(), systemFunc(post_sigma4, vel, dT).y(), systemFunc(post_sigma4, vel, dT).z());
      tf::Vector3 y5(systemFunc(post_sigma5, vel, dT).x(), systemFunc(post_sigma5, vel, dT).y(), systemFunc(post_sigma5, vel, dT).z());
      tf::Vector3 y6(systemFunc(post_sigma6, vel, dT).x(), systemFunc(post_sigma6, vel, dT).y(), systemFunc(post_sigma6, vel, dT).z());

      ROS_DEBUG("**xi(k|k-1)****");
      ROS_DEBUG("y0 = (%.3f, %.3f, %.3f)", y0.x(), y0.y(), y0.z());
      ROS_DEBUG("y1 = (%.3f, %.3f, %.3f)", y1.x(), y1.y(), y1.z());
      ROS_DEBUG("y2 = (%.3f, %.3f, %.3f)", y2.x(), y2.y(), y2.z());
      ROS_DEBUG("y3 = (%.3f, %.3f, %.3f)", y3.x(), y3.y(), y3.z());
      ROS_DEBUG("y4 = (%.3f, %.3f, %.3f)", y4.x(), y4.y(), y4.z());
      ROS_DEBUG("y5 = (%.3f, %.3f, %.3f)", y5.x(), y5.y(), y5.z());
      ROS_DEBUG("y6 = (%.3f, %.3f, %.3f)", y6.x(), y6.y(), y6.z());
      ////3.2 先验估计均值.
      //先验估计.priori(k-1)
      prev_state_.X.setValue((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).x(),
                             (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).y(),
                             angleTrunc((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).z()));
      ROS_DEBUG("***priori(k)***");
      ROS_DEBUG("priori = (%.3f, %.3f, %.3f)", prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
      ////3.3 先验估计方差.
      tf::Matrix3x3 temp0 = vec2Matrix(y0-prev_state_.X, y0-prev_state_.X);
      tf::Matrix3x3 temp1 = vec2Matrix(y1-prev_state_.X, y1-prev_state_.X);
      tf::Matrix3x3 temp2 = vec2Matrix(y2-prev_state_.X, y2-prev_state_.X);
      tf::Matrix3x3 temp3 = vec2Matrix(y3-prev_state_.X, y3-prev_state_.X);
      tf::Matrix3x3 temp4 = vec2Matrix(y4-prev_state_.X, y4-prev_state_.X);
      tf::Matrix3x3 temp5 = vec2Matrix(y5-prev_state_.X, y5-prev_state_.X);
      tf::Matrix3x3 temp6 = vec2Matrix(y6-prev_state_.X, y6-prev_state_.X);

      prev_state_.P = wc0*temp0+wc1.x()*temp1+wc1.y()*temp2+wc1.z()*temp3+
                      wc2.x()*temp4+wc2.y()*temp5+wc2.z()*temp6 +
                      Q_;
      ROS_DEBUG("******");
      ROS_DEBUG("******");
      ROS_DEBUG("prev_P=(%.3f, %.3f, %.3f)", prev_state_.P[0].x(),prev_state_.P[0].y(),prev_state_.P[0].z());
      ROS_DEBUG("prev_P=(%.3f, %.3f, %.3f)", prev_state_.P[1].x(),prev_state_.P[1].y(),prev_state_.P[1].z());
      ROS_DEBUG("prev_P=(%.3f, %.3f, %.3f)", prev_state_.P[2].x(),prev_state_.P[2].y(),prev_state_.P[2].z());

      //4.measurement update.
      ////4.1 prev_state construct Sigma points
      // priori(k)
      tf::Vector3 prev_sigma0(prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
      tf::Vector3 prev_sigma1((prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).x(),
                              (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).y(),
                              angleTrunc((prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).z()));
      tf::Vector3 prev_sigma2((prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).x(),
                              (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).y(),
                              angleTrunc((prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).z()));
      tf::Vector3 prev_sigma3((prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).x(),
                              (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).y(),
                              angleTrunc((prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).z()));
      tf::Vector3 prev_sigma4((prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).x(),
                              (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).y(),
                              angleTrunc((prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).z()));
      tf::Vector3 prev_sigma5((prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).x(),
                              (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).y(),
                              angleTrunc((prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).z()));
      tf::Vector3 prev_sigma6((prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).x(),
                              (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).y(),
                              angleTrunc((prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).z()));
      ROS_DEBUG("******");
      ROS_DEBUG("prev_sigma0=(%.3f, %.3f, %.3f)", prev_sigma0.x(), prev_sigma0.y(), prev_sigma0.z());
      ROS_DEBUG("prev_sigma1=(%.3f, %.3f, %.3f)", prev_sigma1.x(), prev_sigma1.y(), prev_sigma1.z());
      ROS_DEBUG("prev_sigma2=(%.3f, %.3f, %.3f)", prev_sigma2.x(), prev_sigma2.y(), prev_sigma2.z());
      ROS_DEBUG("prev_sigma3=(%.3f, %.3f, %.3f)", prev_sigma3.x(), prev_sigma3.y(), prev_sigma3.z());
      ROS_DEBUG("prev_sigma4=(%.3f, %.3f, %.3f)", prev_sigma4.x(), prev_sigma4.y(), prev_sigma4.z());
      ROS_DEBUG("prev_sigma5=(%.3f, %.3f, %.3f)", prev_sigma5.x(), prev_sigma5.y(), prev_sigma5.z());
      ROS_DEBUG("prev_sigma6=(%.3f, %.3f, %.3f)", prev_sigma6.x(), prev_sigma6.y(), prev_sigma6.z());
      ROS_DEBUG("flag2");
      ////4.2 measurement function.
      ROS_DEBUG("******");
      ROS_DEBUG("measurement_last=(%.3f, %.3f,%.3f)", measu_last_.x(), measu_last_.y(), measu_last_.z());
      tf::Vector3 prev_measu0(measuFunc(QRcoor_curr, prev_sigma0, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma0, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma0, measu_last_).z());
      tf::Vector3 prev_measu1(measuFunc(QRcoor_curr, prev_sigma1, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma1, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma1, measu_last_).z());
      tf::Vector3 prev_measu2(measuFunc(QRcoor_curr, prev_sigma2, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma2, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma2, measu_last_).z());
      tf::Vector3 prev_measu3(measuFunc(QRcoor_curr, prev_sigma3, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma3, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma3, measu_last_).z());
      tf::Vector3 prev_measu4(measuFunc(QRcoor_curr, prev_sigma4, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma4, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma4, measu_last_).z());
      tf::Vector3 prev_measu5(measuFunc(QRcoor_curr, prev_sigma5, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma5, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma5, measu_last_).z());
      tf::Vector3 prev_measu6(measuFunc(QRcoor_curr, prev_sigma6, measu_last_).x(),
                              measuFunc(QRcoor_curr, prev_sigma6, measu_last_).y(),
                              measuFunc(QRcoor_curr, prev_sigma6, measu_last_).z());
      ROS_DEBUG("******");
      ROS_DEBUG("prev_measu0=(%.3f, %.3f, %.3f)", prev_measu0.x(), prev_measu0.y(), prev_measu0.z());
      ROS_DEBUG("prev_measu1=(%.3f, %.3f, %.3f)", prev_measu1.x(), prev_measu1.y(), prev_measu1.z());
      ROS_DEBUG("prev_measu2=(%.3f, %.3f, %.3f)", prev_measu2.x(), prev_measu2.y(), prev_measu2.z());
      ROS_DEBUG("prev_measu3=(%.3f, %.3f, %.3f)", prev_measu3.x(), prev_measu3.y(), prev_measu3.z());
      ROS_DEBUG("prev_measu4=(%.3f, %.3f, %.3f)", prev_measu4.x(), prev_measu4.y(), prev_measu4.z());
      ROS_DEBUG("prev_measu5=(%.3f, %.3f, %.3f)", prev_measu5.x(), prev_measu5.y(), prev_measu5.z());
      ROS_DEBUG("prev_measu6=(%.3f, %.3f, %.3f)", prev_measu6.x(), prev_measu6.y(), prev_measu6.z());
      //// prev_measument
      tf::Vector3 prev_measu((wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).x(),
                             (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).y(),
                             (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).z());
      ROS_DEBUG("******");
      ROS_DEBUG("priori_z = (%.3f, %.3f, %.3f)", prev_measu.x(), prev_measu.y(), prev_measu.z());
    //  ros::shutdown();
      ////4.3 Estimate the covariance of the predicted measurement.
      tf::Matrix3x3 measu_py0 = vec2Matrix(prev_measu0-prev_measu, prev_measu0-prev_measu);
      tf::Matrix3x3 measu_py1 = vec2Matrix(prev_measu1-prev_measu, prev_measu1-prev_measu);
      tf::Matrix3x3 measu_py2 = vec2Matrix(prev_measu2-prev_measu, prev_measu2-prev_measu);
      tf::Matrix3x3 measu_py3 = vec2Matrix(prev_measu3-prev_measu, prev_measu3-prev_measu);
      tf::Matrix3x3 measu_py4 = vec2Matrix(prev_measu4-prev_measu, prev_measu4-prev_measu);
      tf::Matrix3x3 measu_py5 = vec2Matrix(prev_measu5-prev_measu, prev_measu5-prev_measu);
      tf::Matrix3x3 measu_py6 = vec2Matrix(prev_measu6-prev_measu, prev_measu6-prev_measu);

      tf::Matrix3x3 Pzz(wc0*measu_py0+
                       wc1.x()*measu_py1+wc1.y()*measu_py2+wc1.z()*measu_py3+
                       wc2.x()*measu_py4+wc2.y()*measu_py5+wc2.z()*measu_py6+
                       R_);

      ////4.4 Estimate the cross covariance between prev_state and predicted measurement.
      tf::Matrix3x3 pxy0 = vec2Matrix(prev_sigma0-prev_state_.X, prev_measu0-prev_measu);
      tf::Matrix3x3 pxy1 = vec2Matrix(prev_sigma1-prev_state_.X, prev_measu1-prev_measu);
      tf::Matrix3x3 pxy2 = vec2Matrix(prev_sigma2-prev_state_.X, prev_measu2-prev_measu);
      tf::Matrix3x3 pxy3 = vec2Matrix(prev_sigma3-prev_state_.X, prev_measu3-prev_measu);
      tf::Matrix3x3 pxy4 = vec2Matrix(prev_sigma4-prev_state_.X, prev_measu4-prev_measu);
      tf::Matrix3x3 pxy5 = vec2Matrix(prev_sigma5-prev_state_.X, prev_measu5-prev_measu);
      tf::Matrix3x3 pxy6 = vec2Matrix(prev_sigma6-prev_state_.X, prev_measu6-prev_measu);

      tf::Matrix3x3 Pxy(wc0*pxy0+
                        wc1.x()*pxy1+wc1.y()*pxy2+wc1.z()*pxy3+
                        wc2.x()*pxy4+wc2.y()*pxy5+wc2.z()*pxy6);

      ROS_DEBUG("******");
      ROS_DEBUG("Pzz=(%.3f, %.3f, %.3f)", Pzz[0].x(),Pzz[0].y(),Pzz[0].z());
      ROS_DEBUG("Pzz=(%.3f, %.3f, %.3f)", Pzz[1].x(),Pzz[1].y(),Pzz[1].z());
      ROS_DEBUG("Pzz=(%.3f, %.3f, %.3f)", Pzz[2].x(),Pzz[2].y(),Pzz[2].z());

      ROS_DEBUG("Pxy=(%.3f, %.3f, %.3f)", Pxy[0].x(),Pxy[0].y(),Pxy[0].z());
      ROS_DEBUG("Pxy=(%.3f, %.3f, %.3f)", Pxy[1].x(),Pxy[1].y(),Pxy[1].z());
      ROS_DEBUG("Pxy=(%.3f, %.3f, %.3f)", Pxy[2].x(),Pxy[2].y(),Pxy[2].z());
      //// 4.5 measurement update.
      K = Pxy * Pzz.inverse();
      measu_curr.setValue(QRdata.QRtag.x, QRdata.QRtag.y, angleTrunc(QRdata.QRtag.angle));
      ROS_DEBUG("******");
      ROS_DEBUG("measu_current=(%.3f, %.3f, %.3f)", measu_curr.x(), measu_curr.y(), measu_curr.z());
      ROS_DEBUG("\n");
      ROS_DEBUG("K=[%.3f, %.3f, %.3f]", K[0].x(), K[0].y(), K[0].z());
      ROS_DEBUG("K=[%.3f, %.3f, %.3f]", K[1].x(), K[1].y(), K[1].z());
      ROS_DEBUG("K=[%.3f, %.3f, %.3f]", K[2].x(), K[2].y(), K[2].z());

      ROS_DEBUG("\n");
      ROS_DEBUG("dz=(%.4f, %.4f, %.4f)", (measu_curr - prev_measu).x(), (measu_curr - prev_measu).y(), (measu_curr - prev_measu).z());
      post_state_.X = prev_state_.X + K * (measu_curr - prev_measu);
      post_state_.X.setZ(angleTrunc(post_state_.X.z()));

      post_state_.P = prev_state_.P - K * Pzz * K.transpose();
      ROS_DEBUG("posterior = (%.3f, %.3f, %.3f)", post_state_.X.x(), post_state_.X.y(), post_state_.X.z());

   //   measu_last_ = measu_curr;
      t_last_ = QRdata.header.stamp.toSec();

      //使用真实观测到的位姿作为基准计算估计误差．

      tf::Vector3 pose_ref = QRcoor_curr + measu_curr;
      tf::Vector3 pose_err(fabs(post_state_.X.x()) - fabs(pose_ref.x()),
                           fabs(post_state_.X.y()) - fabs(pose_ref.y()),
                           fabs(post_state_.X.z()) - fabs(pose_ref.z()));
      tf::Vector3 odom_err(fabs(raw_odom_.x()) - fabs(pose_ref.x()),
                           fabs(raw_odom_.y()) - fabs(pose_ref.y()),
                           fabs(raw_odom_.z()) - fabs(pose_ref.z()));
      out_pose_error_ << pose_err.x() << " "
                      << pose_err.y() << " "
                      << odom_err.x() << " "
                      << odom_err.y() << std::endl;
      if(debug_)
        out_ekf_ << post_state_.X.x() << ","
               << post_state_.X.y() << ","
               << post_state_.X.z() << ","
               << K.getRow(0).x() << ","
               << K.getRow(1).y() << ","
               << K.getRow(2).z() << ","
               << post_state_.P.getRow(0).x() << ","
               << post_state_.P.getRow(1).y() << ","
               << post_state_.P.getRow(2).z() << ","
               << measu_curr.x() << ","
               << measu_curr.y()
               <<std::endl;
    }

  }

}
void PoseEKF::UKFupdateTest()
{
  while(true)
  {
    waitMilli(20);
    //   ROS_INFO("debug = %s", debug_?"true":"false");
       agvparking_msg::AgvOdom QRdata;
       QRdata.pose.orientation.z = 1.0;
       const unsigned int n = 3;
       static unsigned int counter = 0;
       static tf::Vector3 vel;      //控制量u.
      // vel.setZero();        //清零.
       tf::Matrix3x3 K;
       tf::Vector3 measu_curr;
       measu_curr.setZero();
       boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
       if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(30)))
       {
         QRdata = QRinfo_;
         ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
       //  ROS_INFO("Get QR's data");
         lock.unlock();

         vel.setX(fabs(QRdata.twist.linear.x)<0.001?0.0:QRdata.twist.linear.x);
         vel.setY(fabs(QRdata.twist.linear.y)<0.001?0.0:QRdata.twist.linear.y);
         vel.setZ(fabs(QRdata.twist.angular.z)<0.001?0.0:QRdata.twist.angular.z);

       }
       else
       {
         ROS_WARN("Copy QR's data failed!");
         continue;
       }

       if(QRdata.QRtag.Tagnum)     //进行融合.
       {
         counter++;
         if(!initialized_)
         {
           if(debug_)
           {
             out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_pose.txt");
             if(!out_ekf_.is_open())
               ROS_WARN("'ukf_pose.txt' open failed.");
           }
           t_last_ = QRdata.header.stamp.toSec();
/*           prev_state_.P.setValue(0.01, 0.0, 0.0,
                                  0.0, 0.01, 0.0,
                                  0.0, 0.0, 0.01);
           post_state_.P.setValue(0.001, 0.0, 0.0,
                                  0.0, 0.001, 0.0,
                                  0.0, 0.0, 0.001);
           measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

           Q_.setValue(0.00005, 0.0, 0.0,
                       0.0, 0.00005, 0.0,
                       0.0, 0.0, 0.00005);

           R_.setValue(0.01, 0.0, 0.0,
                       0.0, 0.01, 0.0,
                       0.0, 0.0, 0.01);
           err_P_.setValue(0.01, 0.0, 0.0,
                           0.0, 0.01, 0.0,
                           0.0, 0.0, 0.01);*/

   /*        R_.setValue(QRdata.QRtag.covariance[0], QRdata.QRtag.covariance[1], QRdata.QRtag.covariance[2],
                       QRdata.QRtag.covariance[3], QRdata.QRtag.covariance[4], QRdata.QRtag.covariance[5],
                       QRdata.QRtag.covariance[6], QRdata.QRtag.covariance[7], QRdata.QRtag.covariance[8]);
   */

           prev_state_.P.setValue(0.1, 0.0, 0.0,
                                  0.0, 0.1, 0.0,
                                  0.0, 0.0, 0.1);
           post_state_.P.setValue(0.01, 0.0, 0.0,
                                  0.0, 0.01, 0.0,
                                  0.0, 0.0, 0.01);
           measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

           Q_.setValue(0.01, 0.0, 0.0,
                       0.0, 0.01, 0.0,
                       0.0, 0.0, 0.01);

           R_.setValue(0.1, 0.0, 0.0,
                       0.0, 0.1, 0.0,
                       0.0, 0.0, 0.1);
/*           err_P_.setValue(0.001, 0.0, 0.0,
                           0.0, 0.001, 0.0,
                           0.0, 0.0, 0.001);*/

           initialized_ = true;
           continue;
         }
   //      ROS_INFO("vel=(%.3f, %.3f, %.3f)", vel.x(), vel.y(), vel.z());
         const float dT = fabs(QRdata.header.stamp.toSec() - t_last_);
       //  ROS_INFO("dt = %.3f", dT);
         if(dT < 0)
         {
           ROS_ERROR("we can't estimate past state.");
           continue;
         }

         //四舍五入
         int pre_pose_x = 0, pre_pose_y = 0;
         if(QRdata.pose.position.x >= 0)
         {
           pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 + 5)/10.0);
         }
         else
           pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 - 5)/10.0);
         if(QRdata.pose.position.y >= 0)
         {
           pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 + 5)/10.0);
         }
         else
           pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 - 5)/10.0);

         const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
         const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
      //   const tf::Vector3 dQR(QRcoor_curr);
         QRcoor_ = QRcoor_curr;         //更新.

         prev_state_.X.setValue(QRdata.pose.position.x-ref_.x(),                    //(r2-r1):x
                                QRdata.pose.position.y-ref_.y(),                  //(r2-r1):y
                                tf::getYaw(QRdata.pose.orientation)-ref_.z());    //(r2-r1):yaw //先验估计.
         ref_.setValue(QRdata.pose.position.x, QRdata.pose.position.y, tf::getYaw(QRdata.pose.orientation));

         //UKF
         //1. 权重系数初始化.
         static const float alpha = 0.2;
         static const int kap = 0;
     //    const float kap = factor_;
         static const int beta = 2;
         static const float lamda = (float)(alpha*alpha*(n+kap) - n);      //n=3.
         static const float nc = (float)(n + lamda);
         //均值权重系数
         static const float wm0 = lamda/nc;
         static const tf::Vector3 wm1(1/(2*nc), 1/(2*nc), 1/(2*nc));
         static const tf::Vector3 wm2(1/(2*nc), 1/(2*nc), 1/(2*nc));
         //方差权重系数
         static const float wc0 = wm0 + (1-alpha*alpha+beta);
         static const tf::Vector3 wc1(wm1.x(), wm1.y(), wm1.z());
         static const tf::Vector3 wc2(wm2.x(), wm2.y(), wm2.z());
         static const float n_root = sqrt(nc);

         static const unsigned int L = 10;
         static int nk = 0;
         nk = counter - L + 1;
         //2.post_state构造Sigma点
         ROS_INFO("flag1");
         tf::Vector3 post_sigma0(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
         tf::Vector3 post_sigma1((post_state_.X + n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 post_sigma2((post_state_.X + n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 post_sigma3((post_state_.X + n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).z());
         tf::Vector3 post_sigma4((post_state_.X - n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 post_sigma5((post_state_.X - n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 post_sigma6((post_state_.X - n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).z());

   /*      ROS_INFO("post P=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
                           post_state_.P[0].x(),post_state_.P[0].y(),post_state_.P[0].z(),
                           post_state_.P[1].x(),post_state_.P[1].y(),post_state_.P[1].z(),
                           post_state_.P[2].x(),post_state_.P[2].y(),post_state_.P[2].z());

         ROS_INFO("root vector=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 0).x(),
                                                    rootMatrix(post_state_.P, 0).y(),
                                                    rootMatrix(post_state_.P, 0).z());*/
         //3.时间更新
         ////3.1 状态更新
   /*      tf::Vector3 y0(systemFunc(post_sigma0, vel, dT).x(), systemFunc(post_sigma0, vel, dT).y(), systemFunc(post_sigma0, vel, dT).z());
         tf::Vector3 y1(systemFunc(post_sigma1, vel, dT).x(), systemFunc(post_sigma1, vel, dT).y(), systemFunc(post_sigma1, vel, dT).z());
         tf::Vector3 y2(systemFunc(post_sigma2, vel, dT).x(), systemFunc(post_sigma2, vel, dT).y(), systemFunc(post_sigma2, vel, dT).z());
         tf::Vector3 y3(systemFunc(post_sigma3, vel, dT).x(), systemFunc(post_sigma3, vel, dT).y(), systemFunc(post_sigma3, vel, dT).z());
         tf::Vector3 y4(systemFunc(post_sigma4, vel, dT).x(), systemFunc(post_sigma4, vel, dT).y(), systemFunc(post_sigma4, vel, dT).z());
         tf::Vector3 y5(systemFunc(post_sigma5, vel, dT).x(), systemFunc(post_sigma5, vel, dT).y(), systemFunc(post_sigma5, vel, dT).z());
         tf::Vector3 y6(systemFunc(post_sigma6, vel, dT).x(), systemFunc(post_sigma6, vel, dT).y(), systemFunc(post_sigma6, vel, dT).z());*/

         tf::Vector3 y0(QRdata.pose.position.x - post_sigma0.x(), QRdata.pose.position.y - post_sigma0.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma0.z());
         tf::Vector3 y1(QRdata.pose.position.x - post_sigma1.x(), QRdata.pose.position.y - post_sigma1.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma1.z());
         tf::Vector3 y2(QRdata.pose.position.x - post_sigma2.x(), QRdata.pose.position.y - post_sigma2.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma2.z());
         tf::Vector3 y3(QRdata.pose.position.x - post_sigma3.x(), QRdata.pose.position.y - post_sigma3.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma3.z());
         tf::Vector3 y4(QRdata.pose.position.x - post_sigma4.x(), QRdata.pose.position.y - post_sigma4.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma4.z());
         tf::Vector3 y5(QRdata.pose.position.x - post_sigma5.x(), QRdata.pose.position.y - post_sigma5.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma5.z());
         tf::Vector3 y6(QRdata.pose.position.x - post_sigma6.x(), QRdata.pose.position.y - post_sigma6.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma6.z());

       //  ROS_INFO("y0 = (%.3f, %.3f, %.3f)", y0.x(), y0.y(), y0.z());
         ////3.2 先验估计均值.
   /*      prev_state_.X.setValue((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).x()-post_state_.X.x(),
                                (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).y()-post_state_.X.y(),
                                (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).z()-post_state_.X.z());*/

         prev_state_.X.setValue((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).x(),
                                (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).y(),
                                (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).z());

         ////3.3 先验估计方差.
         tf::Matrix3x3 temp0 = vec2Matrix(y0-prev_state_.X, y0-prev_state_.X);
         tf::Matrix3x3 temp1 = vec2Matrix(y1-prev_state_.X, y1-prev_state_.X);
         tf::Matrix3x3 temp2 = vec2Matrix(y2-prev_state_.X, y2-prev_state_.X);
         tf::Matrix3x3 temp3 = vec2Matrix(y3-prev_state_.X, y3-prev_state_.X);
         tf::Matrix3x3 temp4 = vec2Matrix(y4-prev_state_.X, y4-prev_state_.X);
         tf::Matrix3x3 temp5 = vec2Matrix(y5-prev_state_.X, y5-prev_state_.X);
         tf::Matrix3x3 temp6 = vec2Matrix(y6-prev_state_.X, y6-prev_state_.X);

   /*      tf::Matrix3x3 temp0 = vec2Matrix(y0-post_state_.X-prev_state_.X, y0-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp1 = vec2Matrix(y1-post_state_.X-prev_state_.X, y1-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp2 = vec2Matrix(y2-post_state_.X-prev_state_.X, y2-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp3 = vec2Matrix(y3-post_state_.X-prev_state_.X, y3-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp4 = vec2Matrix(y4-post_state_.X-prev_state_.X, y4-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp5 = vec2Matrix(y5-post_state_.X-prev_state_.X, y5-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp6 = vec2Matrix(y6-post_state_.X-prev_state_.X, y6-post_state_.X-prev_state_.X);*/

         prev_state_.P = wc0*temp0+wc1.x()*temp1+wc1.y()*temp2+wc1.z()*temp3+
                         wc2.x()*temp4+wc2.y()*temp5+wc2.z()*temp6 +
                         Q_;
   /*      ROS_INFO("prev P=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
             prev_state_.P[0].x(),prev_state_.P[0].y(),prev_state_.P[0].z(),
             prev_state_.P[1].x(),prev_state_.P[1].y(),prev_state_.P[1].z(),
             prev_state_.P[2].x(),prev_state_.P[2].y(),prev_state_.P[2].z());*/
         //4.measurement update.
         ////4.1 prev_state construct Sigma points
         tf::Vector3 prev_sigma0(prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
         tf::Vector3 prev_sigma1((prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).x(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).y(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).z());
         tf::Vector3 prev_sigma2((prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).x(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).y(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).z());
         tf::Vector3 prev_sigma3((prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).x(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).y(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).z());
         tf::Vector3 prev_sigma4((prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).x(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).y(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).z());
         tf::Vector3 prev_sigma5((prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).x(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).y(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).z());
         tf::Vector3 prev_sigma6((prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).x(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).y(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).z());
         ROS_INFO("flag2");
         ////4.2 measurement function.
         tf::Vector3 prev_measu0(measuFunc(dQR, prev_sigma0, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma0, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma0, measu_last_).z());
         tf::Vector3 prev_measu1(measuFunc(dQR, prev_sigma1, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma1, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma1, measu_last_).z());
         tf::Vector3 prev_measu2(measuFunc(dQR, prev_sigma2, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma2, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma2, measu_last_).z());
         tf::Vector3 prev_measu3(measuFunc(dQR, prev_sigma3, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma3, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma3, measu_last_).z());
         tf::Vector3 prev_measu4(measuFunc(dQR, prev_sigma4, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma4, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma4, measu_last_).z());
         tf::Vector3 prev_measu5(measuFunc(dQR, prev_sigma5, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma5, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma5, measu_last_).z());
         tf::Vector3 prev_measu6(measuFunc(dQR, prev_sigma6, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma6, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma6, measu_last_).z());

         //// priori_measument(预估测量)
         tf::Vector3 prev_measu((wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).x(),
                                (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).y(),
                                (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).z());

         ////4.3 Estimate the covariance of the predicted measurement.
         tf::Matrix3x3 measu_py0 = vec2Matrix(prev_measu0-prev_measu, prev_measu0-prev_measu);
         tf::Matrix3x3 measu_py1 = vec2Matrix(prev_measu1-prev_measu, prev_measu1-prev_measu);
         tf::Matrix3x3 measu_py2 = vec2Matrix(prev_measu2-prev_measu, prev_measu2-prev_measu);
         tf::Matrix3x3 measu_py3 = vec2Matrix(prev_measu3-prev_measu, prev_measu3-prev_measu);
         tf::Matrix3x3 measu_py4 = vec2Matrix(prev_measu4-prev_measu, prev_measu4-prev_measu);
         tf::Matrix3x3 measu_py5 = vec2Matrix(prev_measu5-prev_measu, prev_measu5-prev_measu);
         tf::Matrix3x3 measu_py6 = vec2Matrix(prev_measu6-prev_measu, prev_measu6-prev_measu);

         tf::Matrix3x3 Py(wc0*measu_py0+
                          wc1.x()*measu_py1+wc1.y()*measu_py2+wc1.z()*measu_py3+
                          wc2.x()*measu_py4+wc2.y()*measu_py4+wc2.z()*measu_py6+
                          R_);
         ////4.4 Estimate the cross covariance between prev_state and predicted measurement.
         tf::Matrix3x3 pxy0 = vec2Matrix(prev_sigma0-prev_state_.X, prev_measu0-prev_measu);
         tf::Matrix3x3 pxy1 = vec2Matrix(prev_sigma1-prev_state_.X, prev_measu1-prev_measu);
         tf::Matrix3x3 pxy2 = vec2Matrix(prev_sigma2-prev_state_.X, prev_measu2-prev_measu);
         tf::Matrix3x3 pxy3 = vec2Matrix(prev_sigma3-prev_state_.X, prev_measu3-prev_measu);
         tf::Matrix3x3 pxy4 = vec2Matrix(prev_sigma4-prev_state_.X, prev_measu4-prev_measu);
         tf::Matrix3x3 pxy5 = vec2Matrix(prev_sigma5-prev_state_.X, prev_measu5-prev_measu);
         tf::Matrix3x3 pxy6 = vec2Matrix(prev_sigma6-prev_state_.X, prev_measu6-prev_measu);

         tf::Matrix3x3 Pxy(wc0*pxy0+
                           wc1.x()*pxy1+wc1.y()*pxy2+wc1.z()*pxy3+
                           wc2.x()*pxy4+wc2.y()*pxy4+wc2.z()*pxy6);
   /*      ROS_INFO("Py=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
                           Py[0].x(),Py[0].y(),Py[0].z(),
                           Py[1].x(),Py[1].y(),Py[1].z(),
                           Py[2].x(),Py[2].y(),Py[2].z());
         ROS_INFO("Pxy=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
                           Pxy[0].x(),Pxy[0].y(),Pxy[0].z(),
                           Pxy[1].x(),Pxy[1].y(),Pxy[1].z(),
                           Pxy[2].x(),Pxy[2].y(),Pxy[2].z());*/
         //// 4.5 measurement update.
         K = Pxy * Py.inverse();
          measu_curr.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
         post_state_.X += prev_state_.X + K * (measu_curr - prev_measu);
         post_state_.P = prev_state_.P - K * Py * K.transpose();

         //sigma points
         tf::Vector3 delta_post((prev_state_.X + K * (measu_curr - prev_measu)).x(),
                                (prev_state_.X + K * (measu_curr - prev_measu)).y(),
                                (prev_state_.X + K * (measu_curr - prev_measu)).z());

         tf::Vector3 error_sigma0(delta_post.x(), delta_post.y(), delta_post.z());
          tf::Vector3 error_sigma1((delta_post + n_root*rootMatrix(post_state_.P, 0)).x(),
                                  (delta_post + n_root*rootMatrix(post_state_.P, 0)).y(),
                                  (delta_post + n_root*rootMatrix(post_state_.P, 0)).z());
          tf::Vector3 error_sigma2((delta_post + n_root*rootMatrix(post_state_.P, 1)).x(),
                                  (delta_post + n_root*rootMatrix(post_state_.P, 1)).y(),
                                  (delta_post + n_root*rootMatrix(post_state_.P, 1)).z());
          tf::Vector3 error_sigma3((delta_post + n_root*rootMatrix(post_state_.P, 2)).x(),
                                  (delta_post + n_root*rootMatrix(post_state_.P, 2)).y(),
                                  (delta_post + n_root*rootMatrix(post_state_.P, 2)).z());
          tf::Vector3 error_sigma4((delta_post - n_root*rootMatrix(post_state_.P, 0)).x(),
                                  (delta_post - n_root*rootMatrix(post_state_.P, 0)).y(),
                                  (delta_post - n_root*rootMatrix(post_state_.P, 0)).z());
          tf::Vector3 error_sigma5((delta_post - n_root*rootMatrix(post_state_.P, 1)).x(),
                                  (delta_post - n_root*rootMatrix(post_state_.P, 1)).y(),
                                  (delta_post - n_root*rootMatrix(post_state_.P, 1)).z());
          tf::Vector3 error_sigma6((delta_post - n_root*rootMatrix(post_state_.P, 2)).x(),
                                  (delta_post - n_root*rootMatrix(post_state_.P, 2)).y(),
                                  (delta_post - n_root*rootMatrix(post_state_.P, 2)).z());

         //post measurement.
          tf::Vector3 post_measu0(measuFunc(dQR, error_sigma0, measu_last_).x(),
                                  measuFunc(dQR, error_sigma0, measu_last_).y(),
                                  measuFunc(dQR, error_sigma0, measu_last_).z());
          tf::Vector3 post_measu1(measuFunc(dQR, error_sigma1, measu_last_).x(),
                                  measuFunc(dQR, error_sigma1, measu_last_).y(),
                                  measuFunc(dQR, error_sigma1, measu_last_).z());
          tf::Vector3 post_measu2(measuFunc(dQR, error_sigma2, measu_last_).x(),
                                  measuFunc(dQR, error_sigma2, measu_last_).y(),
                                  measuFunc(dQR, error_sigma2, measu_last_).z());
          tf::Vector3 post_measu3(measuFunc(dQR, error_sigma3, measu_last_).x(),
                                  measuFunc(dQR, error_sigma3, measu_last_).y(),
                                  measuFunc(dQR, error_sigma3, measu_last_).z());
          tf::Vector3 post_measu4(measuFunc(dQR, error_sigma4, measu_last_).x(),
                                  measuFunc(dQR, error_sigma4, measu_last_).y(),
                                  measuFunc(dQR, error_sigma4, measu_last_).z());
          tf::Vector3 post_measu5(measuFunc(dQR, error_sigma5, measu_last_).x(),
                                  measuFunc(dQR, error_sigma5, measu_last_).y(),
                                  measuFunc(dQR, error_sigma5, measu_last_).z());
          tf::Vector3 post_measu6(measuFunc(dQR, error_sigma6, measu_last_).x(),
                                  measuFunc(dQR, error_sigma6, measu_last_).y(),
                                  measuFunc(dQR, error_sigma6, measu_last_).z());

          //post measurement mean.
          tf::Vector3 post_measu((wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).x(),
                                       (wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).y(),
                                       (wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).z());
          //residual.
          tf::Vector3 error(measu_curr.x() - post_measu.x(),
                               measu_curr.y() - post_measu.y(),
                               measu_curr.z() - post_measu.z());
          ROS_INFO("error = (%.3f, %.3f, %.3f)", error.x(), error.y(), error.z());
          //滑动均值.
          if(nk >= 0)
          {
            //error covariance.
            err_P_ = vec2Matrix(error, error);

            tf::Matrix3x3 Fk = optimMoveAverageFilter(err_P_);

            ROS_INFO("Fk =(%.6f, %.6f, %.6f\n"
                "             %.6f, %.6f, %.6f\n"
                "             %.6f, %.6f, %.6f)",
                              Fk[0].x(),Fk[0].y(),Fk[0].z(),
                              Fk[1].x(),Fk[1].y(),Fk[1].z(),
                              Fk[2].x(),Fk[2].y(),Fk[2].z());
            tf::Matrix3x3 error_P0 = vec2Matrix(prev_measu0-measu_curr+error, prev_measu0-measu_curr+error);
            tf::Matrix3x3 error_P1 = vec2Matrix(prev_measu1-measu_curr+error, prev_measu1-measu_curr+error);
            tf::Matrix3x3 error_P2 = vec2Matrix(prev_measu2-measu_curr+error, prev_measu2-measu_curr+error);
            tf::Matrix3x3 error_P3 = vec2Matrix(prev_measu3-measu_curr+error, prev_measu3-measu_curr+error);
            tf::Matrix3x3 error_P4 = vec2Matrix(prev_measu4-measu_curr+error, prev_measu4-measu_curr+error);
            tf::Matrix3x3 error_P5 = vec2Matrix(prev_measu5-measu_curr+error, prev_measu5-measu_curr+error);
            tf::Matrix3x3 error_P6 = vec2Matrix(prev_measu6-measu_curr+error, prev_measu6-measu_curr+error);

            tf::Matrix3x3 temp(wc0*error_P0+
                             wc1.x()*error_P1+wc1.y()*error_P2+wc1.z()*error_P3+
                             wc2.x()*error_P4+wc2.y()*error_P4+wc2.z()*error_P6);

            R_ = Fk + Py;
            Q_ = K*Fk*K.transpose();
          }



         measu_last_ = measu_curr;
         t_last_ = QRdata.header.stamp.toSec();

         if(debug_)
           out_ekf_ << post_state_.X.x() << ","
                  << post_state_.X.y() << ","
                  << post_state_.X.z() << ","
                  << K.getRow(0).x() << ","
                  << K.getRow(1).y() << ","
                  << K.getRow(2).z() << ","
                  << post_state_.P.getRow(0).x() << ","
                  << post_state_.P.getRow(1).y() << ","
                  << post_state_.P.getRow(2).z() << ","
                  << measu_curr.x() << ","
                  << measu_curr.y()
                  <<std::endl;
       }
       else
       {
   /*      post_state_.X.setX(QRdata.pose.position.x);
         post_state_.X.setY(QRdata.pose.position.y);
         post_state_.X.setZ(QRdata.pose.position.z);*/
       }
  }

}

void PoseEKF::UKFUpdateOdom()
{
    while(true)
    {
   //   ROS_INFO("debug = %s", debug_?"true":"false");
      agvparking_msg::AgvOdom QRdata;
      QRdata.pose.orientation.w = 1.0;
      const unsigned int n = 3;
      static tf::Vector3 vel;      //控制量u.
      tf::Matrix3x3 K;          //增益矩阵.
      tf::Vector3 measu_curr;
      measu_curr.setZero();
      boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
      if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(30)))
      {
        QRdata = QRinfo_;
        ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
      //  ROS_INFO("Get QR's data");
        lock.unlock();

        vel.setX(fabs(QRdata.twist.linear.x)<0.001?0.0:QRdata.twist.linear.x);
        vel.setY(fabs(QRdata.twist.linear.y)<0.001?0.0:QRdata.twist.linear.y);
        vel.setZ(fabs(QRdata.twist.angular.z)<0.001?0.0:QRdata.twist.angular.z);

      }
      else
      {
        ROS_WARN("Copy QR's data failed!");
        continue;
      }
      float fac_x = 3.105;
      float fac_y = 2.92;
      float fac_z = -0.01;
      if(vel.x() < 0)
        fac_x = 3.08;
      if(vel.y() < 0)
        fac_y = 2.9;
      current_time_ = ros::Time::now();
      const float dt = (current_time_ - last_time_).toSec();

      const float dx = vel.x() * dt * fac_x;     //3.09
      const float dy = vel.y() * dt * fac_y;     //3.12
      const float dtheta = vel.z() * dt * fac_z;       //3.1125
      //odometry updates the pose.
      double temp_x, temp_y, temp_z;
      temp_x = ukf_pose_.x() + dx * cos(ukf_pose_.z() + dtheta/2.0) - dy * sin(ukf_pose_.z() + dtheta/2.0);
      temp_y = ukf_pose_.y() + dx * sin(ukf_pose_.z() + dtheta/2.0) + dy * cos(ukf_pose_.z() + dtheta/2.0);
      temp_z = ukf_pose_.z() + dtheta;

      ukf_pose_.setX(temp_x);
      ukf_pose_.setY(temp_y);
      ukf_pose_.setZ(temp_z);

      if(ukf_pose_.z() > M_PI)
        ukf_pose_.setZ(ukf_pose_.z() - 2*M_PI);
      else if(ukf_pose_.z() < -M_PI)
        ukf_pose_.setZ(ukf_pose_.z() + 2*M_PI);
      //raw odometry.
      {
        float factor_x = 3.14;
        float factor_y = 3.12;
        float factor_z = 3.1125;
        if(vel.x() < 0)
          factor_x = 3.10;
        if(vel.y() < 0)
          factor_y = 3.16;

        const float delta_x = vel.x() * dt * factor_x;
        const float delta_y = vel.y() * dt * factor_y;
        const float delta_theta = vel.z() * dt * factor_z;
        //odometry updates the pose.
        double raw_x, raw_y, raw_z;
        raw_x = raw_odom_.x() + delta_x * cos(raw_odom_.z() + delta_theta/2.0) - delta_y * sin(raw_odom_.z() + delta_theta/2.0);
        raw_y = raw_odom_.y() + delta_x * sin(raw_odom_.z() + delta_theta/2.0) + delta_y * cos(raw_odom_.z() + delta_theta/2.0);
        raw_z = raw_odom_.z() + delta_theta;

        raw_odom_.setX(raw_x);
        raw_odom_.setY(raw_y);
        raw_odom_.setZ(raw_z);

        if(raw_odom_.z() > M_PI)
          raw_odom_.setZ(raw_odom_.z() - 2*M_PI);
        else if(raw_odom_.z() < -M_PI)
          raw_odom_.setZ(raw_odom_.z() + 2*M_PI);
        
      }
      if(!initialized_)
      {
        if(debug_)
        {
          out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_pose.txt");
          if(!out_ekf_.is_open())
            ROS_WARN("'ukf_pose.txt' open failed.");
          out_error_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_error.txt");
          if(!out_error_.is_open())
            ROS_WARN("'ukf_error.txt' open failed.");
          out_error_x_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_error_x.txt");
          if(!out_error_x_.is_open())
            ROS_WARN("'ukf_error_.txt' open failed.");
          out_error_y_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_error_y.txt");
          if(!out_error_y_.is_open())
            ROS_WARN("'ukf_error_.txt' open failed.");
          out_pose_error_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_pose_error.txt");
          if(!out_pose_error_.is_open())
            ROS_WARN("'out_pose_error.txt' open failed.");
        }
        t_last_ = QRdata.header.stamp.toSec();

        post_state_.P.setValue(0.001, 0.0, 0.0,
                               0.0, 0.001, 0.0,
                               0.0, 0.0, 0.001);
        post_state_.X.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);

        measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

        Q_.setValue(0.001, 0.0, 0.0,
                    0.0, 0.001, 0.0,
                    0.0, 0.0, 0.001);

        R_.setValue(0.005, 0.0, 0.0,
                    0.0, 0.005, 0.0,
                    0.0, 0.0, 0.005);
        initialized_ = true;
        continue;
      }
 //     ROS_INFO("odometry=(%.4f, %.4f, %.4f)", pose_.x(), pose_.y(),pose_.z());
      if(QRdata.QRtag.Tagnum)     //进行融合.
      {
        //四舍五入,根据里程计计算已经过的DM码的数量.
        int pre_pose_x = 0, pre_pose_y = 0;
        if(QRdata.pose.position.x >= 0)
        {
          pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 + 5)/10.0);
        }
        else
          pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 - 5)/10.0);
        if(QRdata.pose.position.y >= 0)
        {
          pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 + 5)/10.0);
        }
        else
          pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 - 5)/10.0);
   //     if(pre_pose_x == -1)
        if(QRdata.QRtag.Tagnum == 2)
        {
   //       out_ekf_ << "estimator" << std::endl;
  //        ROS_INFO("delta_x1 = %.5f", prev_state_.X + K * (measu_curr - prev_measu));
          ROS_INFO("odometry=(%.4f, %.4f, %.4f)", ukf_pose_.x(), ukf_pose_.y(),ukf_pose_.z());
          ROS_INFO("post_x=(%.4f, %.4f, %.4f)", post_state_.X.x(), post_state_.X.y(),post_state_.X.z());

        }

        ROS_INFO("pre_pose_x = %d, pre_pose_y = %d", pre_pose_x, pre_pose_y);
        const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
        const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
     //   const tf::Vector3 dQR(QRcoor_curr);
        QRcoor_ = QRcoor_curr;         //更新.

        //UKF
        //1. 权重系数初始化.
        static const float alpha = 0.05;
        static const int kap = 0;
    //    const float kap = factor_;
        static const int beta = 2;
        static const float lamda = (float)(alpha*alpha*(n+kap) - n);      //n=3.
        static const float nc = (float)(n + lamda);
        //均值权重系数
        static const float wm0 = lamda/nc;
        static const tf::Vector3 wm1(1/(2*nc), 1/(2*nc), 1/(2*nc));
        static const tf::Vector3 wm2(1/(2*nc), 1/(2*nc), 1/(2*nc));
        //方差权重系数
        static const float wc0 = wm0 + (1-alpha*alpha+beta);
        static const tf::Vector3 wc1(wm1.x(), wm1.y(), wm1.z());
        static const tf::Vector3 wc2(wm2.x(), wm2.y(), wm2.z());
        static const float n_root = sqrt(nc);
        //2.post_state构造Sigma点
  //      ROS_INFO("flag1");
        //sigma(n;k-1) = post_x(k-1)+root((n+gama)P);
        tf::Vector3 post_sigma0(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
        tf::Vector3 post_sigma1((post_state_.X + n_root*rootMatrix(post_state_.P, 0)).x(),
                                (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).y(),
                                (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).z());
        tf::Vector3 post_sigma2((post_state_.X + n_root*rootMatrix(post_state_.P, 1)).x(),
                                (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).y(),
                                (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).z());
        tf::Vector3 post_sigma3((post_state_.X + n_root*rootMatrix(post_state_.P, 2)).x(),
                                (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).y(),
                                (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).z());
        tf::Vector3 post_sigma4((post_state_.X - n_root*rootMatrix(post_state_.P, 0)).x(),
                                (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).y(),
                                (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).z());
        tf::Vector3 post_sigma5((post_state_.X - n_root*rootMatrix(post_state_.P, 1)).x(),
                                (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).y(),
                                (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).z());
        tf::Vector3 post_sigma6((post_state_.X - n_root*rootMatrix(post_state_.P, 2)).x(),
                                (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).y(),
                                (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).z());

        ROS_INFO("post P=(%.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f)",
                          post_state_.P[0].x(),post_state_.P[0].y(),post_state_.P[0].z(),
                          post_state_.P[1].x(),post_state_.P[1].y(),post_state_.P[1].z(),
                          post_state_.P[2].x(),post_state_.P[2].y(),post_state_.P[2].z());

        ROS_INFO("root vector=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 0).x(),
                                                   rootMatrix(post_state_.P, 0).y(),
                                                   rootMatrix(post_state_.P, 0).z());
        //3.时间更新
        ////3.1 状态更新
  /*      tf::Vector3 y0(systemFunc(post_sigma0, vel, dT).x(), systemFunc(post_sigma0, vel, dT).y(), systemFunc(post_sigma0, vel, dT).z());
        tf::Vector3 y1(systemFunc(post_sigma1, vel, dT).x(), systemFunc(post_sigma1, vel, dT).y(), systemFunc(post_sigma1, vel, dT).z());
        tf::Vector3 y2(systemFunc(post_sigma2, vel, dT).x(), systemFunc(post_sigma2, vel, dT).y(), systemFunc(post_sigma2, vel, dT).z());
        tf::Vector3 y3(systemFunc(post_sigma3, vel, dT).x(), systemFunc(post_sigma3, vel, dT).y(), systemFunc(post_sigma3, vel, dT).z());
        tf::Vector3 y4(systemFunc(post_sigma4, vel, dT).x(), systemFunc(post_sigma4, vel, dT).y(), systemFunc(post_sigma4, vel, dT).z());
        tf::Vector3 y5(systemFunc(post_sigma5, vel, dT).x(), systemFunc(post_sigma5, vel, dT).y(), systemFunc(post_sigma5, vel, dT).z());
        tf::Vector3 y6(systemFunc(post_sigma6, vel, dT).x(), systemFunc(post_sigma6, vel, dT).y(), systemFunc(post_sigma6, vel, dT).z());*/
        //y = systemFunc(post_sigma(i)) - post_sigma(i);
       /* tf::Vector3 y0(ukf_pose_.x() - post_sigma0.x(), ukf_pose_.y() - post_sigma0.y(), ukf_pose_.z()- post_sigma0.z());
        tf::Vector3 y1(ukf_pose_.x() - post_sigma1.x(), ukf_pose_.y() - post_sigma1.y(), ukf_pose_.z()- post_sigma1.z());
        tf::Vector3 y2(ukf_pose_.x() - post_sigma2.x(), ukf_pose_.y() - post_sigma2.y(), ukf_pose_.z()- post_sigma2.z());
        tf::Vector3 y3(ukf_pose_.x() - post_sigma3.x(), ukf_pose_.y() - post_sigma3.y(), ukf_pose_.z()- post_sigma3.z());
        tf::Vector3 y4(ukf_pose_.x() - post_sigma4.x(), ukf_pose_.y() - post_sigma4.y(), ukf_pose_.z()- post_sigma4.z());
        tf::Vector3 y5(ukf_pose_.x() - post_sigma5.x(), ukf_pose_.y() - post_sigma5.y(), ukf_pose_.z()- post_sigma5.z());
        tf::Vector3 y6(ukf_pose_.x() - post_sigma6.x(), ukf_pose_.y() - post_sigma6.y(), ukf_pose_.z()- post_sigma6.z());
*/
        tf::Vector3 y0(QRdata.pose.position.x - post_sigma0.x(), QRdata.pose.position.y - post_sigma0.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma0.z());
        tf::Vector3 y1(QRdata.pose.position.x - post_sigma1.x(), QRdata.pose.position.y - post_sigma1.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma1.z());
        tf::Vector3 y2(QRdata.pose.position.x - post_sigma2.x(), QRdata.pose.position.y - post_sigma2.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma2.z());
        tf::Vector3 y3(QRdata.pose.position.x - post_sigma3.x(), QRdata.pose.position.y - post_sigma3.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma3.z());
        tf::Vector3 y4(QRdata.pose.position.x - post_sigma4.x(), QRdata.pose.position.y - post_sigma4.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma4.z());
        tf::Vector3 y5(QRdata.pose.position.x - post_sigma5.x(), QRdata.pose.position.y - post_sigma5.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma5.z());
        tf::Vector3 y6(QRdata.pose.position.x - post_sigma6.x(), QRdata.pose.position.y - post_sigma6.y(), tf::getYaw(QRdata.pose.orientation)- post_sigma6.z());

        ROS_INFO("y0 = (%.3f, %.3f, %.3f)", y0.x(), y0.y(), y0.z());
        ////3.2 先验估计均值.

        prev_state_.X.setValue((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).x(),
                               (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).y(),
                               (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).z());
/*        prev_state_.X.setValue(pose_.x() - post_state_.X.x(),
                               pose_.x() - post_state_.X.y(),
                               pose_.x() - post_state_.X.z());*/
        ROS_INFO("prev_state = (%.3f, %.3f, %.3f)", prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
        ////3.3 先验估计方差.
        tf::Matrix3x3 temp0 = vec2Matrix(y0-prev_state_.X, y0-prev_state_.X);
        tf::Matrix3x3 temp1 = vec2Matrix(y1-prev_state_.X, y1-prev_state_.X);
        tf::Matrix3x3 temp2 = vec2Matrix(y2-prev_state_.X, y2-prev_state_.X);
        tf::Matrix3x3 temp3 = vec2Matrix(y3-prev_state_.X, y3-prev_state_.X);
        tf::Matrix3x3 temp4 = vec2Matrix(y4-prev_state_.X, y4-prev_state_.X);
        tf::Matrix3x3 temp5 = vec2Matrix(y5-prev_state_.X, y5-prev_state_.X);
        tf::Matrix3x3 temp6 = vec2Matrix(y6-prev_state_.X, y6-prev_state_.X);

  /*      tf::Matrix3x3 temp0 = vec2Matrix(y0-post_state_.X-prev_state_.X, y0-post_state_.X-prev_state_.X);
        tf::Matrix3x3 temp1 = vec2Matrix(y1-post_state_.X-prev_state_.X, y1-post_state_.X-prev_state_.X);
        tf::Matrix3x3 temp2 = vec2Matrix(y2-post_state_.X-prev_state_.X, y2-post_state_.X-prev_state_.X);
        tf::Matrix3x3 temp3 = vec2Matrix(y3-post_state_.X-prev_state_.X, y3-post_state_.X-prev_state_.X);
        tf::Matrix3x3 temp4 = vec2Matrix(y4-post_state_.X-prev_state_.X, y4-post_state_.X-prev_state_.X);
        tf::Matrix3x3 temp5 = vec2Matrix(y5-post_state_.X-prev_state_.X, y5-post_state_.X-prev_state_.X);
        tf::Matrix3x3 temp6 = vec2Matrix(y6-post_state_.X-prev_state_.X, y6-post_state_.X-prev_state_.X);*/

        prev_state_.P = wc0*temp0+wc1.x()*temp1+wc1.y()*temp2+wc1.z()*temp3+
                        wc2.x()*temp4+wc2.y()*temp5+wc2.z()*temp6 +
                        Q_;
  /*      ROS_INFO("prev P=(%.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f)",
            prev_state_.P[0].x(),prev_state_.P[0].y(),prev_state_.P[0].z(),
            prev_state_.P[1].x(),prev_state_.P[1].y(),prev_state_.P[1].z(),
            prev_state_.P[2].x(),prev_state_.P[2].y(),prev_state_.P[2].z());*/
        //4.measurement update.
        ////4.1 prev_state construct Sigma points
        tf::Vector3 prev_sigma0(prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
        tf::Vector3 prev_sigma1((prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).x(),
                                (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).y(),
                                (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).z());
        tf::Vector3 prev_sigma2((prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).x(),
                                (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).y(),
                                (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).z());
        tf::Vector3 prev_sigma3((prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).x(),
                                (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).y(),
                                (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).z());
        tf::Vector3 prev_sigma4((prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).x(),
                                (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).y(),
                                (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).z());
        tf::Vector3 prev_sigma5((prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).x(),
                                (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).y(),
                                (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).z());
        tf::Vector3 prev_sigma6((prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).x(),
                                (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).y(),
                                (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).z());
//        ROS_INFO("flag2");

        ////4.2 measurement function.
        tf::Vector3 prev_measu0(measuFunc(dQR, prev_sigma0, measu_last_).x(),
                                measuFunc(dQR, prev_sigma0, measu_last_).y(),
                                measuFunc(dQR, prev_sigma0, measu_last_).z());
        tf::Vector3 prev_measu1(measuFunc(dQR, prev_sigma1, measu_last_).x(),
                                measuFunc(dQR, prev_sigma1, measu_last_).y(),
                                measuFunc(dQR, prev_sigma1, measu_last_).z());
        tf::Vector3 prev_measu2(measuFunc(dQR, prev_sigma2, measu_last_).x(),
                                measuFunc(dQR, prev_sigma2, measu_last_).y(),
                                measuFunc(dQR, prev_sigma2, measu_last_).z());
        tf::Vector3 prev_measu3(measuFunc(dQR, prev_sigma3, measu_last_).x(),
                                measuFunc(dQR, prev_sigma3, measu_last_).y(),
                                measuFunc(dQR, prev_sigma3, measu_last_).z());
        tf::Vector3 prev_measu4(measuFunc(dQR, prev_sigma4, measu_last_).x(),
                                measuFunc(dQR, prev_sigma4, measu_last_).y(),
                                measuFunc(dQR, prev_sigma4, measu_last_).z());
        tf::Vector3 prev_measu5(measuFunc(dQR, prev_sigma5, measu_last_).x(),
                                measuFunc(dQR, prev_sigma5, measu_last_).y(),
                                measuFunc(dQR, prev_sigma5, measu_last_).z());
        tf::Vector3 prev_measu6(measuFunc(dQR, prev_sigma6, measu_last_).x(),
                                measuFunc(dQR, prev_sigma6, measu_last_).y(),
                                measuFunc(dQR, prev_sigma6, measu_last_).z());

        //// prev_measument
        tf::Vector3 prev_measu((wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).x(),
                               (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).y(),
                               (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).z());
       // ros::shutdown();
        ////4.3 Estimate the covariance of the predicted measurement.
        tf::Matrix3x3 measu_py0 = vec2Matrix(prev_measu0-prev_measu, prev_measu0-prev_measu);
        tf::Matrix3x3 measu_py1 = vec2Matrix(prev_measu1-prev_measu, prev_measu1-prev_measu);
        tf::Matrix3x3 measu_py2 = vec2Matrix(prev_measu2-prev_measu, prev_measu2-prev_measu);
        tf::Matrix3x3 measu_py3 = vec2Matrix(prev_measu3-prev_measu, prev_measu3-prev_measu);
        tf::Matrix3x3 measu_py4 = vec2Matrix(prev_measu4-prev_measu, prev_measu4-prev_measu);
        tf::Matrix3x3 measu_py5 = vec2Matrix(prev_measu5-prev_measu, prev_measu5-prev_measu);
        tf::Matrix3x3 measu_py6 = vec2Matrix(prev_measu6-prev_measu, prev_measu6-prev_measu);

        tf::Matrix3x3 Py(wc0*measu_py0+
                         wc1.x()*measu_py1+wc1.y()*measu_py2+wc1.z()*measu_py3+
                         wc2.x()*measu_py4+wc2.y()*measu_py5+wc2.z()*measu_py6+
                         R_);
        ////4.4 Estimate the cross covariance between prev_state and predicted measurement.
        tf::Matrix3x3 pxy0 = vec2Matrix(prev_sigma0-prev_state_.X, prev_measu0-prev_measu);
        tf::Matrix3x3 pxy1 = vec2Matrix(prev_sigma1-prev_state_.X, prev_measu1-prev_measu);
        tf::Matrix3x3 pxy2 = vec2Matrix(prev_sigma2-prev_state_.X, prev_measu2-prev_measu);
        tf::Matrix3x3 pxy3 = vec2Matrix(prev_sigma3-prev_state_.X, prev_measu3-prev_measu);
        tf::Matrix3x3 pxy4 = vec2Matrix(prev_sigma4-prev_state_.X, prev_measu4-prev_measu);
        tf::Matrix3x3 pxy5 = vec2Matrix(prev_sigma5-prev_state_.X, prev_measu5-prev_measu);
        tf::Matrix3x3 pxy6 = vec2Matrix(prev_sigma6-prev_state_.X, prev_measu6-prev_measu);

        tf::Matrix3x3 Pxy(wc0*pxy0+
                          wc1.x()*pxy1+wc1.y()*pxy2+wc1.z()*pxy3+
                          wc2.x()*pxy4+wc2.y()*pxy5+wc2.z()*pxy6);
  /*      ROS_INFO("Py=(%.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f)",
                          Py[0].x(),Py[0].y(),Py[0].z(),
                          Py[1].x(),Py[1].y(),Py[1].z(),
                          Py[2].x(),Py[2].y(),Py[2].z());
        ROS_INFO("Pxy=(%.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f\n"
            "             %.3f, %.3f, %.3f)",
                          Pxy[0].x(),Pxy[0].y(),Pxy[0].z(),
                          Pxy[1].x(),Pxy[1].y(),Pxy[1].z(),
                          Pxy[2].x(),Pxy[2].y(),Pxy[2].z());*/
        //// 4.5 measurement update.
        K = Pxy * Py.inverse();
        measu_curr.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
        ROS_INFO("current measurement = (%.3f, %.3f, %.3f)", measu_curr.x(), measu_curr.y(), measu_curr.z());

        post_state_.X += prev_state_.X + K * (measu_curr - prev_measu);
        post_state_.P = prev_state_.P - K * Py * K.transpose();

        measu_last_ = measu_curr;
        ukf_pose_.setValue(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());  //更新里程计位姿.
/*        if(debug_)
          out_ekf_ << post_state_.X.x() << ","
                 << post_state_.X.y() << ","
                 << post_state_.X.z() << ","
                 << K.getRow(0).x() << ","
                 << K.getRow(1).y() << ","
                 << K.getRow(2).z() << ","
                 << post_state_.P.getRow(0).x() << ","
                 << post_state_.P.getRow(1).y() << ","
                 << post_state_.P.getRow(2).z() << ","
                 << measu_curr.x() << ","
                 << measu_curr.y()
                 <<std::endl;*/

        //估计过程噪声
        //sigma points
        tf::Vector3 delta_post((prev_state_.X + K * (measu_curr - prev_measu)).x(),
                               (prev_state_.X + K * (measu_curr - prev_measu)).y(),
                               (prev_state_.X + K * (measu_curr - prev_measu)).z());

        tf::Vector3 error_sigma0(delta_post.x(), delta_post.y(), delta_post.z());
         tf::Vector3 error_sigma1((delta_post + n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 error_sigma2((delta_post + n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 error_sigma3((delta_post + n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 2)).z());
         tf::Vector3 error_sigma4((delta_post - n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 error_sigma5((delta_post - n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 error_sigma6((delta_post - n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 2)).z());

        //post measurement.
         tf::Vector3 post_measu0(measuFunc(dQR, error_sigma0, measu_last_).x(),
                                 measuFunc(dQR, error_sigma0, measu_last_).y(),
                                 measuFunc(dQR, error_sigma0, measu_last_).z());
         tf::Vector3 post_measu1(measuFunc(dQR, error_sigma1, measu_last_).x(),
                                 measuFunc(dQR, error_sigma1, measu_last_).y(),
                                 measuFunc(dQR, error_sigma1, measu_last_).z());
         tf::Vector3 post_measu2(measuFunc(dQR, error_sigma2, measu_last_).x(),
                                 measuFunc(dQR, error_sigma2, measu_last_).y(),
                                 measuFunc(dQR, error_sigma2, measu_last_).z());
         tf::Vector3 post_measu3(measuFunc(dQR, error_sigma3, measu_last_).x(),
                                 measuFunc(dQR, error_sigma3, measu_last_).y(),
                                 measuFunc(dQR, error_sigma3, measu_last_).z());
         tf::Vector3 post_measu4(measuFunc(dQR, error_sigma4, measu_last_).x(),
                                 measuFunc(dQR, error_sigma4, measu_last_).y(),
                                 measuFunc(dQR, error_sigma4, measu_last_).z());
         tf::Vector3 post_measu5(measuFunc(dQR, error_sigma5, measu_last_).x(),
                                 measuFunc(dQR, error_sigma5, measu_last_).y(),
                                 measuFunc(dQR, error_sigma5, measu_last_).z());
         tf::Vector3 post_measu6(measuFunc(dQR, error_sigma6, measu_last_).x(),
                                 measuFunc(dQR, error_sigma6, measu_last_).y(),
                                 measuFunc(dQR, error_sigma6, measu_last_).z());

         //post measurement mean.
         tf::Vector3 post_measu((wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).x(),
                                      (wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).y(),
                                      (wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).z());
         //residual.
         error_.setValue(measu_curr.x() - post_measu.x(),
                              measu_curr.y() - post_measu.y(),
                              measu_curr.z() - post_measu.z());

         ROS_INFO("error = (%.3f, %.3f, %.3f)", error_.x(), error_.y(), error_.z());

         out_error_ << error_.x() << ","
                   << error_.y() << ","
                   << error_.z() << ","
                   <<std::endl;

         if(fabs(vel.x()) > fabs(vel.y()))
           out_error_y_ << error_.y() << std::endl;
         else if(fabs(vel.y()) > fabs(vel.x()))
           out_error_x_ << error_.x() << std::endl;
         else
         {
           out_error_y_ << error_.y() << std::endl;
           out_error_x_ << error_.x() << std::endl;
         }
         //使用真实观测到的位姿作为基准计算估计误差．
/*         tf::Vector3 pose_ref = QRcoor_curr + measu_curr;
         tf::Vector3 pose_err = post_state_.X - pose_ref;
         tf::Vector3 odom_err = raw_odom_ - pose_ref;*/

         tf::Vector3 pose_ref = QRcoor_curr + measu_curr;
         tf::Vector3 pose_err(fabs(post_state_.X.x()) - fabs(pose_ref.x()),
                              fabs(post_state_.X.y()) - fabs(pose_ref.y()),
                              fabs(post_state_.X.z()) - fabs(pose_ref.z()));
         tf::Vector3 odom_err(fabs(raw_odom_.x()) - fabs(pose_ref.x()),
                              fabs(raw_odom_.y()) - fabs(pose_ref.y()),
                              fabs(raw_odom_.z()) - fabs(pose_ref.z()));
         out_pose_error_ << pose_err.x() << " "
                         << pose_err.y() << " "
                         << odom_err.x() << " "
                         << odom_err.y() << std::endl;

      }
      else
      {
  /*      post_state_.X.setX(QRdata.pose.position.x);
        post_state_.X.setY(QRdata.pose.position.y);
        post_state_.X.setZ(QRdata.pose.position.z);*/
      }
  //    ROS_INFO("post_state = (%.3f, %.3f, %.3f)", post_state_.X.x(), post_state_.X.y(), post_state_.X.z());

      if(debug_)
        out_ekf_ << ukf_pose_.x() << ","
               << ukf_pose_.y() << ","
               << ukf_pose_.z() << ","
               << K.getRow(0).x() << ","
               << K.getRow(1).y() << ","
               << K.getRow(2).z() << ","
               << post_state_.P.getRow(0).x() << ","
               << post_state_.P.getRow(1).y() << ","
               << post_state_.P.getRow(2).z() << ","
               << measu_curr.x() << ","
               << measu_curr.y() <<","
               << post_state_.X.x() << ","
               << post_state_.X.y() << ","
               << post_state_.X.z() << ","
               << raw_odom_.x() << ","
               << raw_odom_.y() << ","
               << raw_odom_.z() << ","
               << error_.x() << ","
               << error_.y() << ","
               << error_.z() << ","
               <<std::endl;

      waitMilli(20);
      last_time_ = current_time_;     //update last time.
    }
}

void PoseEKF::UKFAdjust()
{

  while(true)
      {
     //   ROS_INFO("debug = %s", debug_?"true":"false");
        agvparking_msg::AgvOdom QRdata;
        QRdata.pose.orientation.z = 1.0;
        const unsigned int n = 3;
        static tf::Vector3 vel;      //控制量u.
        tf::Matrix3x3 K;          //增益矩阵.
        tf::Vector3 measu_curr;
        measu_curr.setZero();
        boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
        if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(30)))
        {
          QRdata = QRinfo_;
          ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
        //  ROS_INFO("Get QR's data");
          lock.unlock();

          vel.setX(fabs(QRdata.twist.linear.x)<0.001?0.0:QRdata.twist.linear.x);
          vel.setY(fabs(QRdata.twist.linear.y)<0.001?0.0:QRdata.twist.linear.y);
          vel.setZ(fabs(QRdata.twist.angular.z)<0.001?0.0:QRdata.twist.angular.z);

        }
        else
        {
          ROS_WARN("Copy QR's data failed!");
          continue;
        }
        if(!initialized_)
        {
          if(debug_)
          {
            out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/ukf_pose.txt");
            if(!out_ekf_.is_open())
              ROS_WARN("'ukf_pose.txt' open failed.");
          }
          t_last_ = QRdata.header.stamp.toSec();

          post_state_.P.setValue(0.001, 0.0, 0.0,
                                 0.0, 0.001, 0.0,
                                 0.0, 0.0, 0.001);
          post_state_.X.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);

          measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

          Q_.setValue(0.000001, 0.0, 0.0,
                      0.0, 0.000001, 0.0,
                      0.0, 0.0, 0.000001);

          R_.setValue(1.5, 0.0, 0.0,
                      0.0, 1.5, 0.0,
                      0.0, 0.0, 1.5);
          initialized_ = true;
          continue;
        }
   //     ROS_INFO("odometry=(%.4f, %.4f, %.4f)", pose_.x(), pose_.y(),pose_.z());
        if(QRdata.QRtag.Tagnum)     //进行融合.
        {
          //四舍五入,根据里程计计算已经过的DM码的数量.
          int pre_pose_x = 0, pre_pose_y = 0;
          if(QRdata.pose.position.x >= 0)
          {
            pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 + 5)/10.0);
          }
          else
            pre_pose_x = (int)((QRdata.pose.position.x/QRdist*10 - 5)/10.0);
          if(QRdata.pose.position.y >= 0)
          {
            pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 + 5)/10.0);
          }
          else
            pre_pose_y = (int)((QRdata.pose.position.y/QRdist*10 - 5)/10.0);
     //     if(pre_pose_x == -1)
          if(QRdata.QRtag.Tagnum == 2)
          {
     //       out_ekf_ << "estimator" << std::endl;
    //        ROS_INFO("delta_x1 = %.5f", prev_state_.X + K * (measu_curr - prev_measu));
            ROS_INFO("odometry=(%.4f, %.4f, %.4f)", ekf_pose_.x(), ekf_pose_.y(), ekf_pose_.z());
            ROS_INFO("post_x=(%.4f, %.4f, %.4f)", post_state_.X.x(), post_state_.X.y(),post_state_.X.z());

          }

          ROS_INFO("pre_pose_x = %d, pre_pose_y = %d", pre_pose_x, pre_pose_y);
          const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
          const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
       //   const tf::Vector3 dQR(QRcoor_curr);
          QRcoor_ = QRcoor_curr;         //更新.

          //UKF
          //1. 权重系数初始化.
          static const float alpha = 0.05;
          static const int kap = 0;
      //    const float kap = factor_;
          static const int beta = 2;
          static const float lamda = (float)(alpha*alpha*(n+kap) - n);      //n=3.
          static const float nc = (float)(n + lamda);
          //均值权重系数
          static const float wm0 = lamda/nc;
          static const tf::Vector3 wm1(1/(2*nc), 1/(2*nc), 1/(2*nc));
          static const tf::Vector3 wm2(1/(2*nc), 1/(2*nc), 1/(2*nc));
          //方差权重系数
          static const float wc0 = wm0 + (1-alpha*alpha+beta);
          static const tf::Vector3 wc1(wm1.x(), wm1.y(), wm1.z());
          static const tf::Vector3 wc2(wm2.x(), wm2.y(), wm2.z());
          static const float n_root = sqrt(nc);
          //2.post_state构造Sigma点
    //      ROS_INFO("flag1");
          //sigma(n;k-1) = post_x(k-1)+root((n+gama)P);
          tf::Vector3 post_sigma0(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
          tf::Vector3 post_sigma1((post_state_.X + n_root*rootMatrix(post_state_.P, 0)).x(),
                                  (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).y(),
                                  (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).z());
          tf::Vector3 post_sigma2((post_state_.X + n_root*rootMatrix(post_state_.P, 1)).x(),
                                  (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).y(),
                                  (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).z());
          tf::Vector3 post_sigma3((post_state_.X + n_root*rootMatrix(post_state_.P, 2)).x(),
                                  (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).y(),
                                  (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).z());
          tf::Vector3 post_sigma4((post_state_.X - n_root*rootMatrix(post_state_.P, 0)).x(),
                                  (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).y(),
                                  (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).z());
          tf::Vector3 post_sigma5((post_state_.X - n_root*rootMatrix(post_state_.P, 1)).x(),
                                  (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).y(),
                                  (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).z());
          tf::Vector3 post_sigma6((post_state_.X - n_root*rootMatrix(post_state_.P, 2)).x(),
                                  (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).y(),
                                  (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).z());

  /*        ROS_INFO("post P=(%.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f)",
                            post_state_.P[0].x(),post_state_.P[0].y(),post_state_.P[0].z(),
                            post_state_.P[1].x(),post_state_.P[1].y(),post_state_.P[1].z(),
                            post_state_.P[2].x(),post_state_.P[2].y(),post_state_.P[2].z());

          ROS_INFO("root vector=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 0).x(),
                                                     rootMatrix(post_state_.P, 0).y(),
                                                     rootMatrix(post_state_.P, 0).z());*/
          //3.时间更新
          ////3.1 状态更新
    /*      tf::Vector3 y0(systemFunc(post_sigma0, vel, dT).x(), systemFunc(post_sigma0, vel, dT).y(), systemFunc(post_sigma0, vel, dT).z());
          tf::Vector3 y1(systemFunc(post_sigma1, vel, dT).x(), systemFunc(post_sigma1, vel, dT).y(), systemFunc(post_sigma1, vel, dT).z());
          tf::Vector3 y2(systemFunc(post_sigma2, vel, dT).x(), systemFunc(post_sigma2, vel, dT).y(), systemFunc(post_sigma2, vel, dT).z());
          tf::Vector3 y3(systemFunc(post_sigma3, vel, dT).x(), systemFunc(post_sigma3, vel, dT).y(), systemFunc(post_sigma3, vel, dT).z());
          tf::Vector3 y4(systemFunc(post_sigma4, vel, dT).x(), systemFunc(post_sigma4, vel, dT).y(), systemFunc(post_sigma4, vel, dT).z());
          tf::Vector3 y5(systemFunc(post_sigma5, vel, dT).x(), systemFunc(post_sigma5, vel, dT).y(), systemFunc(post_sigma5, vel, dT).z());
          tf::Vector3 y6(systemFunc(post_sigma6, vel, dT).x(), systemFunc(post_sigma6, vel, dT).y(), systemFunc(post_sigma6, vel, dT).z());*/
          //y = systemFunc(post_sigma(i)) - post_sigma(i);
          tf::Vector3 y0(QRdata.pose.position.x - post_sigma0.x(), QRdata.pose.position.y - post_sigma0.y(), ekf_pose_.z()- post_sigma0.z());
          tf::Vector3 y1(QRdata.pose.position.x - post_sigma1.x(), QRdata.pose.position.y - post_sigma1.y(), ekf_pose_.z()- post_sigma1.z());
          tf::Vector3 y2(QRdata.pose.position.x - post_sigma2.x(), QRdata.pose.position.y - post_sigma2.y(), ekf_pose_.z()- post_sigma2.z());
          tf::Vector3 y3(QRdata.pose.position.x - post_sigma3.x(), QRdata.pose.position.y - post_sigma3.y(), ekf_pose_.z()- post_sigma3.z());
          tf::Vector3 y4(QRdata.pose.position.x - post_sigma4.x(), QRdata.pose.position.y - post_sigma4.y(), ekf_pose_.z()- post_sigma4.z());
          tf::Vector3 y5(QRdata.pose.position.x - post_sigma5.x(), QRdata.pose.position.y - post_sigma5.y(), ekf_pose_.z()- post_sigma5.z());
          tf::Vector3 y6(QRdata.pose.position.x - post_sigma6.x(), QRdata.pose.position.y - post_sigma6.y(), ekf_pose_.z()- post_sigma6.z());

          ROS_INFO("y0 = (%.3f, %.3f, %.3f)", y0.x(), y0.y(), y0.z());
          ////3.2 先验估计均值.

          prev_state_.X.setValue((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).x(),
                                 (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).y(),
                                 (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).z());
  /*        prev_state_.X.setValue(pose_.x() - post_state_.X.x(),
                                 pose_.x() - post_state_.X.y(),
                                 pose_.x() - post_state_.X.z());*/
          ROS_INFO("prev_state = (%.3f, %.3f, %.3f)", prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
          ////3.3 先验估计方差.
          tf::Matrix3x3 temp0 = vec2Matrix(y0-prev_state_.X, y0-prev_state_.X);
          tf::Matrix3x3 temp1 = vec2Matrix(y1-prev_state_.X, y1-prev_state_.X);
          tf::Matrix3x3 temp2 = vec2Matrix(y2-prev_state_.X, y2-prev_state_.X);
          tf::Matrix3x3 temp3 = vec2Matrix(y3-prev_state_.X, y3-prev_state_.X);
          tf::Matrix3x3 temp4 = vec2Matrix(y4-prev_state_.X, y4-prev_state_.X);
          tf::Matrix3x3 temp5 = vec2Matrix(y5-prev_state_.X, y5-prev_state_.X);
          tf::Matrix3x3 temp6 = vec2Matrix(y6-prev_state_.X, y6-prev_state_.X);

    /*      tf::Matrix3x3 temp0 = vec2Matrix(y0-post_state_.X-prev_state_.X, y0-post_state_.X-prev_state_.X);
          tf::Matrix3x3 temp1 = vec2Matrix(y1-post_state_.X-prev_state_.X, y1-post_state_.X-prev_state_.X);
          tf::Matrix3x3 temp2 = vec2Matrix(y2-post_state_.X-prev_state_.X, y2-post_state_.X-prev_state_.X);
          tf::Matrix3x3 temp3 = vec2Matrix(y3-post_state_.X-prev_state_.X, y3-post_state_.X-prev_state_.X);
          tf::Matrix3x3 temp4 = vec2Matrix(y4-post_state_.X-prev_state_.X, y4-post_state_.X-prev_state_.X);
          tf::Matrix3x3 temp5 = vec2Matrix(y5-post_state_.X-prev_state_.X, y5-post_state_.X-prev_state_.X);
          tf::Matrix3x3 temp6 = vec2Matrix(y6-post_state_.X-prev_state_.X, y6-post_state_.X-prev_state_.X);*/

          prev_state_.P = wc0*temp0+wc1.x()*temp1+wc1.y()*temp2+wc1.z()*temp3+
                          wc2.x()*temp4+wc2.y()*temp5+wc2.z()*temp6 +
                          Q_;
    /*      ROS_INFO("prev P=(%.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f)",
              prev_state_.P[0].x(),prev_state_.P[0].y(),prev_state_.P[0].z(),
              prev_state_.P[1].x(),prev_state_.P[1].y(),prev_state_.P[1].z(),
              prev_state_.P[2].x(),prev_state_.P[2].y(),prev_state_.P[2].z());*/
          //4.measurement update.
          ////4.1 prev_state construct Sigma points
          tf::Vector3 prev_sigma0(prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
          tf::Vector3 prev_sigma1((prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).x(),
                                  (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).y(),
                                  (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).z());
          tf::Vector3 prev_sigma2((prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).x(),
                                  (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).y(),
                                  (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).z());
          tf::Vector3 prev_sigma3((prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).x(),
                                  (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).y(),
                                  (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).z());
          tf::Vector3 prev_sigma4((prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).x(),
                                  (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).y(),
                                  (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).z());
          tf::Vector3 prev_sigma5((prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).x(),
                                  (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).y(),
                                  (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).z());
          tf::Vector3 prev_sigma6((prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).x(),
                                  (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).y(),
                                  (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).z());
  //        ROS_INFO("flag2");

          ////4.2 measurement function.
          tf::Vector3 prev_measu0(measuFunc(dQR, prev_sigma0, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma0, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma0, measu_last_).z());
          tf::Vector3 prev_measu1(measuFunc(dQR, prev_sigma1, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma1, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma1, measu_last_).z());
          tf::Vector3 prev_measu2(measuFunc(dQR, prev_sigma2, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma2, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma2, measu_last_).z());
          tf::Vector3 prev_measu3(measuFunc(dQR, prev_sigma3, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma3, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma3, measu_last_).z());
          tf::Vector3 prev_measu4(measuFunc(dQR, prev_sigma4, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma4, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma4, measu_last_).z());
          tf::Vector3 prev_measu5(measuFunc(dQR, prev_sigma5, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma5, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma5, measu_last_).z());
          tf::Vector3 prev_measu6(measuFunc(dQR, prev_sigma6, measu_last_).x(),
                                  measuFunc(dQR, prev_sigma6, measu_last_).y(),
                                  measuFunc(dQR, prev_sigma6, measu_last_).z());

          //// prev_measument
          tf::Vector3 prev_measu((wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).x(),
                                 (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).y(),
                                 (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).z());

          ////4.3 Estimate the covariance of the predicted measurement.
          tf::Matrix3x3 measu_py0 = vec2Matrix(prev_measu0-prev_measu, prev_measu0-prev_measu);
          tf::Matrix3x3 measu_py1 = vec2Matrix(prev_measu1-prev_measu, prev_measu1-prev_measu);
          tf::Matrix3x3 measu_py2 = vec2Matrix(prev_measu2-prev_measu, prev_measu2-prev_measu);
          tf::Matrix3x3 measu_py3 = vec2Matrix(prev_measu3-prev_measu, prev_measu3-prev_measu);
          tf::Matrix3x3 measu_py4 = vec2Matrix(prev_measu4-prev_measu, prev_measu4-prev_measu);
          tf::Matrix3x3 measu_py5 = vec2Matrix(prev_measu5-prev_measu, prev_measu5-prev_measu);
          tf::Matrix3x3 measu_py6 = vec2Matrix(prev_measu6-prev_measu, prev_measu6-prev_measu);

          tf::Matrix3x3 Py(wc0*measu_py0+
                           wc1.x()*measu_py1+wc1.y()*measu_py2+wc1.z()*measu_py3+
                           wc2.x()*measu_py4+wc2.y()*measu_py4+wc2.z()*measu_py6+
                           R_);
          ////4.4 Estimate the cross covariance between prev_state and predicted measurement.
          tf::Matrix3x3 pxy0 = vec2Matrix(prev_sigma0-prev_state_.X, prev_measu0-prev_measu);
          tf::Matrix3x3 pxy1 = vec2Matrix(prev_sigma1-prev_state_.X, prev_measu1-prev_measu);
          tf::Matrix3x3 pxy2 = vec2Matrix(prev_sigma2-prev_state_.X, prev_measu2-prev_measu);
          tf::Matrix3x3 pxy3 = vec2Matrix(prev_sigma3-prev_state_.X, prev_measu3-prev_measu);
          tf::Matrix3x3 pxy4 = vec2Matrix(prev_sigma4-prev_state_.X, prev_measu4-prev_measu);
          tf::Matrix3x3 pxy5 = vec2Matrix(prev_sigma5-prev_state_.X, prev_measu5-prev_measu);
          tf::Matrix3x3 pxy6 = vec2Matrix(prev_sigma6-prev_state_.X, prev_measu6-prev_measu);

          tf::Matrix3x3 Pxy(wc0*pxy0+
                            wc1.x()*pxy1+wc1.y()*pxy2+wc1.z()*pxy3+
                            wc2.x()*pxy4+wc2.y()*pxy4+wc2.z()*pxy6);
    /*      ROS_INFO("Py=(%.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f)",
                            Py[0].x(),Py[0].y(),Py[0].z(),
                            Py[1].x(),Py[1].y(),Py[1].z(),
                            Py[2].x(),Py[2].y(),Py[2].z());
          ROS_INFO("Pxy=(%.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f\n"
              "             %.3f, %.3f, %.3f)",
                            Pxy[0].x(),Pxy[0].y(),Pxy[0].z(),
                            Pxy[1].x(),Pxy[1].y(),Pxy[1].z(),
                            Pxy[2].x(),Pxy[2].y(),Pxy[2].z());*/
          //// 4.5 measurement update.
          K = Pxy * Py.inverse();
          measu_curr.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
          tf::Vector3 displacement_odom = prev_state_.X + K * (measu_curr - prev_measu); //修正后的里程计变化量.
          post_state_.X += displacement_odom;
          post_state_.P = prev_state_.P - K * Py * K.transpose();

          measu_last_ = measu_curr;
          ekf_pose_.setValue(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());  //更新里程计位姿.
  /*        if(debug_)
            out_ekf_ << post_state_.X.x() << ","
                   << post_state_.X.y() << ","
                   << post_state_.X.z() << ","
                   << K.getRow(0).x() << ","
                   << K.getRow(1).y() << ","
                   << K.getRow(2).z() << ","
                   << post_state_.P.getRow(0).x() << ","
                   << post_state_.P.getRow(1).y() << ","
                   << post_state_.P.getRow(2).z() << ","
                   << measu_curr.x() << ","
                   << measu_curr.y()
                   <<std::endl;*/
        }
        else
        {
    /*      post_state_.X.setX(QRdata.pose.position.x);
          post_state_.X.setY(QRdata.pose.position.y);
          post_state_.X.setZ(QRdata.pose.position.z);*/
        }
    //    ROS_INFO("post_state = (%.3f, %.3f, %.3f)", post_state_.X.x(), post_state_.X.y(), post_state_.X.z());

        if(debug_)
          out_ekf_ << ekf_pose_.x() << ","
                 << ekf_pose_.y() << ","
                 << ekf_pose_.z() << ","
                 << K.getRow(0).x() << ","
                 << K.getRow(1).y() << ","
                 << K.getRow(2).z() << ","
                 << post_state_.P.getRow(0).x() << ","
                 << post_state_.P.getRow(1).y() << ","
                 << post_state_.P.getRow(2).z() << ","
                 << measu_curr.x() << ","
                 << measu_curr.y() <<","
                 << raw_odom_.x() << ","
                 << raw_odom_.y() << ","
                 << raw_odom_.z() << ","
                 <<std::endl;

        waitMilli(20);
        last_time_ = current_time_;     //update last time.
      }
}

void PoseEKF::AUKFUpdateOdom()
{
  while(true)
  {
    //   ROS_INFO("debug = %s", debug_?"true":"false");
    static unsigned int counter = 0;
    agvparking_msg::AgvOdom QRdata;
       QRdata.pose.orientation.z = 1.0;
       const unsigned int n = 3;
       static tf::Vector3 vel;      //控制量u.
       tf::Matrix3x3 K;          //增益矩阵.
       tf::Vector3 measu_curr;
       measu_curr.setZero();
       boost::unique_lock<boost::timed_mutex> lock(QRmutex_, boost::try_to_lock);
       if(lock.owns_lock() || lock.try_lock_for(boost::chrono::milliseconds(30)))
       {
         QRdata = QRinfo_;
         ROS_DEBUG_ONCE("Get QR's data: %s", QRdata.child_frame_id.c_str());
       //  ROS_INFO("Get QR's data");
         lock.unlock();

         vel.setX(fabs(QRdata.twist.linear.x)<0.001?0.0:QRdata.twist.linear.x);
         vel.setY(fabs(QRdata.twist.linear.y)<0.001?0.0:QRdata.twist.linear.y);
         vel.setZ(fabs(QRdata.twist.angular.z)<0.001?0.0:QRdata.twist.angular.z);

       }
       else
       {
         ROS_WARN("Copy QR's data failed!");
         continue;
       }
       current_time_ = ros::Time::now();
       const float dt = (current_time_ - last_time_).toSec();
       const float dx = vel.x() * dt * 3.09;     //3.09
       const float dy = vel.y() * dt * 3.3;     //3.12
       const float dtheta = vel.z() * dt * 3.125;       //3.1125
       //odometry updates the pose.
       double temp_x = 0.0, temp_y = 0.0, temp_z = 0.0;
       temp_x = aukf_pose_.x() + dx * cos(aukf_pose_.z() + dtheta/2.0) - dy * sin(aukf_pose_.z() + dtheta/2.0);
       temp_y = aukf_pose_.y() + dx * sin(aukf_pose_.z() + dtheta/2.0) + dy * cos(aukf_pose_.z() + dtheta/2.0);
       temp_z = aukf_pose_.z() + dtheta;

       aukf_pose_.setX(temp_x);
       aukf_pose_.setY(temp_y);
       aukf_pose_.setZ(temp_z);

       if(aukf_pose_.z() > M_PI)
         aukf_pose_.setZ(aukf_pose_.z() - 2*M_PI);
       else if(aukf_pose_.z() < -M_PI)
         aukf_pose_.setZ(aukf_pose_.z() + 2*M_PI);
       //raw odometry.
/*       {
         const float delta_x = vel.x() * dt * 2.8;
         const float delta_y = vel.y() * dt * 2.8;
         const float delta_theta = vel.z() * dt * 1.0;
         //odometry updates the pose.
         double raw_x, raw_y, raw_z;
         raw_x = raw_odom_.x() + delta_x * cos(raw_odom_.z() + delta_theta/2.0) - delta_y * sin(raw_odom_.z() + delta_theta/2.0);
         raw_y = raw_odom_.y() + delta_x * sin(raw_odom_.z() + delta_theta/2.0) + delta_y * cos(raw_odom_.z() + delta_theta/2.0);
         raw_z = raw_odom_.z() + delta_theta;

         raw_odom_.setX(raw_x);
         raw_odom_.setY(raw_y);
         raw_odom_.setZ(raw_z);

         if(raw_odom_.z() > M_PI)
           raw_odom_.setZ(raw_odom_.z() - 2*M_PI);
         else if(raw_odom_.z() < -M_PI)
           raw_odom_.setZ(raw_odom_.z() + 2*M_PI);

       }*/
       if(!initialized_)
       {
         if(debug_)
         {
           out_ekf_.open("/home/paul/agvParking_ws/src/agvparking_metapackage/pose_ekf/aukf_pose.txt");
           if(!out_ekf_.is_open())
             ROS_WARN("'ukf_pose.txt' open failed.");
         }
         t_last_ = QRdata.header.stamp.toSec();

         post_state_.P.setValue(0.001, 0.0, 0.0,
                                0.0, 0.001, 0.0,
                                0.0, 0.0, 0.001);
         post_state_.X.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);

         measu_last_.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);       //上一次的观测量.

         Q_.setValue(0.00001, 0.0, 0.0,
                     0.0, 0.00001, 0.0,
                     0.0, 0.0, 0.00001);

         R_.setValue(2.0, 0.0, 0.0,
                     0.0, 2.0, 0.0,
                     0.0, 0.0, 2.0);
         initialized_ = true;
         continue;
       }
  //     ROS_INFO("odometry=(%.4f, %.4f, %.4f)", pose_.x(), pose_.y(),pose_.z());
       if(QRdata.QRtag.Tagnum)     //进行融合.
       {
         //四舍五入,根据里程计计算已经过的DM码的数量.
         int pre_pose_x = 0, pre_pose_y = 0;
         if(aukf_pose_.x() >= 0)
         {
           pre_pose_x = (int)((aukf_pose_.x()/QRdist*10 + 5)/10.0);
         }
         else
           pre_pose_x = (int)((aukf_pose_.x()/QRdist*10 - 5)/10.0);
         if(aukf_pose_.y() >= 0)
         {
           pre_pose_y = (int)((aukf_pose_.y()/QRdist*10 + 5)/10.0);
         }
         else
           pre_pose_y = (int)((aukf_pose_.y()/QRdist*10 - 5)/10.0);
    //     if(pre_pose_x == -1)
         if(QRdata.QRtag.Tagnum == 2)
         {
    //       out_ekf_ << "estimator" << std::endl;
   //        ROS_INFO("delta_x1 = %.5f", prev_state_.X + K * (measu_curr - prev_measu));
           ROS_INFO("odometry=(%.4f, %.4f, %.4f)", aukf_pose_.x(), aukf_pose_.y(), aukf_pose_.z());
           ROS_INFO("post_x=(%.4f, %.4f, %.4f)", post_state_.X.x(), post_state_.X.y(),post_state_.X.z());

         }

         ROS_INFO("pre_pose_x = %d, pre_pose_y = %d", pre_pose_x, pre_pose_y);
         const tf::Vector3 QRcoor_curr((float)pre_pose_x * QRdist, (float)pre_pose_y * QRdist, 0.0);  //当前的QR坐标.
         const tf::Vector3 dQR(QRcoor_curr - QRcoor_);    //QR1->QR2.
      //   const tf::Vector3 dQR(QRcoor_curr);
         QRcoor_ = QRcoor_curr;         //更新.

         //UKF
         //1. 权重系数初始化.
         static const float alpha = 0.05;
         static const int kap = 0;
     //    const float kap = factor_;
         static const int beta = 2;
         static const float lamda = (float)(alpha*alpha*(n+kap) - n);      //n=3.
         static const float nc = (float)(n + lamda);
         //均值权重系数
         static const float wm0 = lamda/nc;
         static const tf::Vector3 wm1(1/(2*nc), 1/(2*nc), 1/(2*nc));
         static const tf::Vector3 wm2(1/(2*nc), 1/(2*nc), 1/(2*nc));
         //方差权重系数
         static const float wc0 = wm0 + (1-alpha*alpha+beta);
         static const tf::Vector3 wc1(wm1.x(), wm1.y(), wm1.z());
         static const tf::Vector3 wc2(wm2.x(), wm2.y(), wm2.z());
         static const float n_root = sqrt(nc);

         //自适应参数
         static const unsigned int L = 10;
         static int nk = 0;
         nk = counter - L + 1;
         //2.post_state构造Sigma点
   //      ROS_INFO("flag1");
         //sigma(n;k-1) = post_x(k-1)+root((n+gama)P);
         tf::Vector3 post_sigma0(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
         tf::Vector3 post_sigma1((post_state_.X + n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 post_sigma2((post_state_.X + n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 post_sigma3((post_state_.X + n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (post_state_.X + n_root*rootMatrix(post_state_.P, 2)).z());
         tf::Vector3 post_sigma4((post_state_.X - n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 post_sigma5((post_state_.X - n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 post_sigma6((post_state_.X - n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (post_state_.X - n_root*rootMatrix(post_state_.P, 2)).z());

 /*        ROS_INFO("post P=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
                           post_state_.P[0].x(),post_state_.P[0].y(),post_state_.P[0].z(),
                           post_state_.P[1].x(),post_state_.P[1].y(),post_state_.P[1].z(),
                           post_state_.P[2].x(),post_state_.P[2].y(),post_state_.P[2].z());

         ROS_INFO("root vector=(%.3f, %.3f, %.3f)", rootMatrix(post_state_.P, 0).x(),
                                                    rootMatrix(post_state_.P, 0).y(),
                                                    rootMatrix(post_state_.P, 0).z());*/
         //3.时间更新
         ////3.1 状态更新
   /*      tf::Vector3 y0(systemFunc(post_sigma0, vel, dT).x(), systemFunc(post_sigma0, vel, dT).y(), systemFunc(post_sigma0, vel, dT).z());
         tf::Vector3 y1(systemFunc(post_sigma1, vel, dT).x(), systemFunc(post_sigma1, vel, dT).y(), systemFunc(post_sigma1, vel, dT).z());
         tf::Vector3 y2(systemFunc(post_sigma2, vel, dT).x(), systemFunc(post_sigma2, vel, dT).y(), systemFunc(post_sigma2, vel, dT).z());
         tf::Vector3 y3(systemFunc(post_sigma3, vel, dT).x(), systemFunc(post_sigma3, vel, dT).y(), systemFunc(post_sigma3, vel, dT).z());
         tf::Vector3 y4(systemFunc(post_sigma4, vel, dT).x(), systemFunc(post_sigma4, vel, dT).y(), systemFunc(post_sigma4, vel, dT).z());
         tf::Vector3 y5(systemFunc(post_sigma5, vel, dT).x(), systemFunc(post_sigma5, vel, dT).y(), systemFunc(post_sigma5, vel, dT).z());
         tf::Vector3 y6(systemFunc(post_sigma6, vel, dT).x(), systemFunc(post_sigma6, vel, dT).y(), systemFunc(post_sigma6, vel, dT).z());*/
         //y = systemFunc(post_sigma(i)) - post_sigma(i);
         tf::Vector3 y0(aukf_pose_.x() - post_sigma0.x(), aukf_pose_.y() - post_sigma0.y(), aukf_pose_.z()- post_sigma0.z());
         tf::Vector3 y1(aukf_pose_.x() - post_sigma1.x(), aukf_pose_.y() - post_sigma1.y(), aukf_pose_.z()- post_sigma1.z());
         tf::Vector3 y2(aukf_pose_.x() - post_sigma2.x(), aukf_pose_.y() - post_sigma2.y(), aukf_pose_.z()- post_sigma2.z());
         tf::Vector3 y3(aukf_pose_.x() - post_sigma3.x(), aukf_pose_.y() - post_sigma3.y(), aukf_pose_.z()- post_sigma3.z());
         tf::Vector3 y4(aukf_pose_.x() - post_sigma4.x(), aukf_pose_.y() - post_sigma4.y(), aukf_pose_.z()- post_sigma4.z());
         tf::Vector3 y5(aukf_pose_.x() - post_sigma5.x(), aukf_pose_.y() - post_sigma5.y(), aukf_pose_.z()- post_sigma5.z());
         tf::Vector3 y6(aukf_pose_.x() - post_sigma6.x(), aukf_pose_.y() - post_sigma6.y(), aukf_pose_.z()- post_sigma6.z());

         ROS_INFO("y0 = (%.3f, %.3f, %.3f)", y0.x(), y0.y(), y0.z());
         ////3.2 先验估计均值.

         prev_state_.X.setValue((wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).x(),
                                (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).y(),
                                (wm0*y0+wm1.x()*y1+wm1.y()*y2+wm1.z()*y3+wm2.x()*y4+wm2.y()*y5+wm2.z()*y6).z());
 /*        prev_state_.X.setValue(pose_.x() - post_state_.X.x(),
                                pose_.x() - post_state_.X.y(),
                                pose_.x() - post_state_.X.z());*/
         ROS_INFO("post_state1 = (%.3f, %.3f, %.3f)", post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
         ROS_INFO("prev_state = (%.3f, %.3f, %.3f)", prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
         ////3.3 先验估计方差.
         tf::Matrix3x3 temp0 = vec2Matrix(y0-prev_state_.X, y0-prev_state_.X);
         tf::Matrix3x3 temp1 = vec2Matrix(y1-prev_state_.X, y1-prev_state_.X);
         tf::Matrix3x3 temp2 = vec2Matrix(y2-prev_state_.X, y2-prev_state_.X);
         tf::Matrix3x3 temp3 = vec2Matrix(y3-prev_state_.X, y3-prev_state_.X);
         tf::Matrix3x3 temp4 = vec2Matrix(y4-prev_state_.X, y4-prev_state_.X);
         tf::Matrix3x3 temp5 = vec2Matrix(y5-prev_state_.X, y5-prev_state_.X);
         tf::Matrix3x3 temp6 = vec2Matrix(y6-prev_state_.X, y6-prev_state_.X);

   /*      tf::Matrix3x3 temp0 = vec2Matrix(y0-post_state_.X-prev_state_.X, y0-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp1 = vec2Matrix(y1-post_state_.X-prev_state_.X, y1-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp2 = vec2Matrix(y2-post_state_.X-prev_state_.X, y2-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp3 = vec2Matrix(y3-post_state_.X-prev_state_.X, y3-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp4 = vec2Matrix(y4-post_state_.X-prev_state_.X, y4-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp5 = vec2Matrix(y5-post_state_.X-prev_state_.X, y5-post_state_.X-prev_state_.X);
         tf::Matrix3x3 temp6 = vec2Matrix(y6-post_state_.X-prev_state_.X, y6-post_state_.X-prev_state_.X);*/

         prev_state_.P = wc0*temp0+wc1.x()*temp1+wc1.y()*temp2+wc1.z()*temp3+
                         wc2.x()*temp4+wc2.y()*temp5+wc2.z()*temp6 +
                         Q_;
   /*      ROS_INFO("prev P=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
             prev_state_.P[0].x(),prev_state_.P[0].y(),prev_state_.P[0].z(),
             prev_state_.P[1].x(),prev_state_.P[1].y(),prev_state_.P[1].z(),
             prev_state_.P[2].x(),prev_state_.P[2].y(),prev_state_.P[2].z());*/
         //4.measurement update.
         ////4.1 prev_state construct Sigma points
         tf::Vector3 prev_sigma0(prev_state_.X.x(), prev_state_.X.y(), prev_state_.X.z());
         tf::Vector3 prev_sigma1((prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).x(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).y(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 0)).z());
         tf::Vector3 prev_sigma2((prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).x(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).y(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 1)).z());
         tf::Vector3 prev_sigma3((prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).x(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).y(),
                                 (prev_state_.X + n_root*rootMatrix(prev_state_.P, 2)).z());
         tf::Vector3 prev_sigma4((prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).x(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).y(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 0)).z());
         tf::Vector3 prev_sigma5((prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).x(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).y(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 1)).z());
         tf::Vector3 prev_sigma6((prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).x(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).y(),
                                 (prev_state_.X - n_root*rootMatrix(prev_state_.P, 2)).z());
 //        ROS_INFO("flag2");

         ////4.2 measurement function.
         tf::Vector3 prev_measu0(measuFunc(dQR, prev_sigma0, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma0, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma0, measu_last_).z());
         tf::Vector3 prev_measu1(measuFunc(dQR, prev_sigma1, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma1, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma1, measu_last_).z());
         tf::Vector3 prev_measu2(measuFunc(dQR, prev_sigma2, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma2, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma2, measu_last_).z());
         tf::Vector3 prev_measu3(measuFunc(dQR, prev_sigma3, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma3, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma3, measu_last_).z());
         tf::Vector3 prev_measu4(measuFunc(dQR, prev_sigma4, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma4, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma4, measu_last_).z());
         tf::Vector3 prev_measu5(measuFunc(dQR, prev_sigma5, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma5, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma5, measu_last_).z());
         tf::Vector3 prev_measu6(measuFunc(dQR, prev_sigma6, measu_last_).x(),
                                 measuFunc(dQR, prev_sigma6, measu_last_).y(),
                                 measuFunc(dQR, prev_sigma6, measu_last_).z());

         //// prev_measument
         tf::Vector3 prev_measu((wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).x(),
                                (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).y(),
                                (wm0*prev_measu0+wm1.x()*prev_measu1+wm1.y()*prev_measu2+wm1.z()*prev_measu3+wm2.x()*prev_measu4+wm2.y()*prev_measu5+wm2.z()*prev_measu6).z());

         ////4.3 Estimate the covariance of the predicted measurement.
         tf::Matrix3x3 measu_py0 = vec2Matrix(prev_measu0-prev_measu, prev_measu0-prev_measu);
         tf::Matrix3x3 measu_py1 = vec2Matrix(prev_measu1-prev_measu, prev_measu1-prev_measu);
         tf::Matrix3x3 measu_py2 = vec2Matrix(prev_measu2-prev_measu, prev_measu2-prev_measu);
         tf::Matrix3x3 measu_py3 = vec2Matrix(prev_measu3-prev_measu, prev_measu3-prev_measu);
         tf::Matrix3x3 measu_py4 = vec2Matrix(prev_measu4-prev_measu, prev_measu4-prev_measu);
         tf::Matrix3x3 measu_py5 = vec2Matrix(prev_measu5-prev_measu, prev_measu5-prev_measu);
         tf::Matrix3x3 measu_py6 = vec2Matrix(prev_measu6-prev_measu, prev_measu6-prev_measu);

         tf::Matrix3x3 Py(wc0*measu_py0+
                          wc1.x()*measu_py1+wc1.y()*measu_py2+wc1.z()*measu_py3+
                          wc2.x()*measu_py4+wc2.y()*measu_py4+wc2.z()*measu_py6+
                          R_);
         ////4.4 Estimate the cross covariance between prev_state and predicted measurement.
         tf::Matrix3x3 pxy0 = vec2Matrix(prev_sigma0-prev_state_.X, prev_measu0-prev_measu);
         tf::Matrix3x3 pxy1 = vec2Matrix(prev_sigma1-prev_state_.X, prev_measu1-prev_measu);
         tf::Matrix3x3 pxy2 = vec2Matrix(prev_sigma2-prev_state_.X, prev_measu2-prev_measu);
         tf::Matrix3x3 pxy3 = vec2Matrix(prev_sigma3-prev_state_.X, prev_measu3-prev_measu);
         tf::Matrix3x3 pxy4 = vec2Matrix(prev_sigma4-prev_state_.X, prev_measu4-prev_measu);
         tf::Matrix3x3 pxy5 = vec2Matrix(prev_sigma5-prev_state_.X, prev_measu5-prev_measu);
         tf::Matrix3x3 pxy6 = vec2Matrix(prev_sigma6-prev_state_.X, prev_measu6-prev_measu);

         tf::Matrix3x3 Pxy(wc0*pxy0+
                           wc1.x()*pxy1+wc1.y()*pxy2+wc1.z()*pxy3+
                           wc2.x()*pxy4+wc2.y()*pxy4+wc2.z()*pxy6);
   /*      ROS_INFO("Py=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
                           Py[0].x(),Py[0].y(),Py[0].z(),
                           Py[1].x(),Py[1].y(),Py[1].z(),
                           Py[2].x(),Py[2].y(),Py[2].z());
         ROS_INFO("Pxy=(%.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f\n"
             "             %.3f, %.3f, %.3f)",
                           Pxy[0].x(),Pxy[0].y(),Pxy[0].z(),
                           Pxy[1].x(),Pxy[1].y(),Pxy[1].z(),
                           Pxy[2].x(),Pxy[2].y(),Pxy[2].z());*/
         //// 4.5 measurement update.
         K = Pxy * Py.inverse();
         measu_curr.setValue(QRdata.QRtag.x, QRdata.QRtag.y, QRdata.QRtag.angle);
         post_state_.X += prev_state_.X + K * (measu_curr - prev_measu);
         post_state_.P = prev_state_.P - K * Py * K.transpose();

         measu_last_ = measu_curr;
         ROS_INFO("post_state2 = (%.3f, %.3f, %.3f)", post_state_.X.x(), post_state_.X.y(), post_state_.X.z());
       //  pose_.setValue(post_state_.X.x(), post_state_.X.y(), post_state_.X.z());  //更新里程计位姿.
/*         pose_.setX(post_state_.X.x());
         pose_.setY(post_state_.X.y());
         pose_.setZ(post_state_.X.z());*/
         aukf_pose_ = post_state_.X;
         ROS_INFO("pose2 = (%.3f, %.3f, %.3f)", aukf_pose_.x(), aukf_pose_.y(), aukf_pose_.z());
        //估计过程噪声
        //sigma points
        tf::Vector3 delta_post((prev_state_.X + K * (measu_curr - prev_measu)).x(),
                               (prev_state_.X + K * (measu_curr - prev_measu)).y(),
                               (prev_state_.X + K * (measu_curr - prev_measu)).z());

        tf::Vector3 error_sigma0(delta_post.x(), delta_post.y(), delta_post.z());
         tf::Vector3 error_sigma1((delta_post + n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 error_sigma2((delta_post + n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 error_sigma3((delta_post + n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (delta_post + n_root*rootMatrix(post_state_.P, 2)).z());
         tf::Vector3 error_sigma4((delta_post - n_root*rootMatrix(post_state_.P, 0)).x(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 0)).y(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 0)).z());
         tf::Vector3 error_sigma5((delta_post - n_root*rootMatrix(post_state_.P, 1)).x(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 1)).y(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 1)).z());
         tf::Vector3 error_sigma6((delta_post - n_root*rootMatrix(post_state_.P, 2)).x(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 2)).y(),
                                 (delta_post - n_root*rootMatrix(post_state_.P, 2)).z());

        //post measurement.
         tf::Vector3 post_measu0(measuFunc(dQR, error_sigma0, measu_last_).x(),
                                 measuFunc(dQR, error_sigma0, measu_last_).y(),
                                 measuFunc(dQR, error_sigma0, measu_last_).z());
         tf::Vector3 post_measu1(measuFunc(dQR, error_sigma1, measu_last_).x(),
                                 measuFunc(dQR, error_sigma1, measu_last_).y(),
                                 measuFunc(dQR, error_sigma1, measu_last_).z());
         tf::Vector3 post_measu2(measuFunc(dQR, error_sigma2, measu_last_).x(),
                                 measuFunc(dQR, error_sigma2, measu_last_).y(),
                                 measuFunc(dQR, error_sigma2, measu_last_).z());
         tf::Vector3 post_measu3(measuFunc(dQR, error_sigma3, measu_last_).x(),
                                 measuFunc(dQR, error_sigma3, measu_last_).y(),
                                 measuFunc(dQR, error_sigma3, measu_last_).z());
         tf::Vector3 post_measu4(measuFunc(dQR, error_sigma4, measu_last_).x(),
                                 measuFunc(dQR, error_sigma4, measu_last_).y(),
                                 measuFunc(dQR, error_sigma4, measu_last_).z());
         tf::Vector3 post_measu5(measuFunc(dQR, error_sigma5, measu_last_).x(),
                                 measuFunc(dQR, error_sigma5, measu_last_).y(),
                                 measuFunc(dQR, error_sigma5, measu_last_).z());
         tf::Vector3 post_measu6(measuFunc(dQR, error_sigma6, measu_last_).x(),
                                 measuFunc(dQR, error_sigma6, measu_last_).y(),
                                 measuFunc(dQR, error_sigma6, measu_last_).z());

         //post measurement mean.
         tf::Vector3 post_measu((wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).x(),
                                      (wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).y(),
                                      (wm0*post_measu0+wm1.x()*post_measu1+wm1.y()*post_measu2+wm1.z()*post_measu3+wm2.x()*post_measu4+wm2.y()*post_measu5+wm2.z()*post_measu6).z());
         //residual.
         tf::Vector3 error(measu_curr.x() - post_measu.x(),
                              measu_curr.y() - post_measu.y(),
                              measu_curr.z() - post_measu.z());

        ROS_INFO("delta_post=(%.5f, %.5f, %.5f)", delta_post.x(), delta_post.y(), delta_post.z());
        ROS_INFO("post_z=(%.5f, %.5f, %.5f)", measuFunc(dQR, delta_post, measu_last_).x(), measuFunc(dQR, delta_post, measu_last_).y(), measuFunc(dQR, delta_post, measu_last_).z());
/*         tf::Vector3 error(measu_curr.x() - measuFunc(dQR, delta_post, measu_last_).x(),
                           measu_curr.y() - measuFunc(dQR, delta_post, measu_last_).y(),
                           measu_curr.z() - measuFunc(dQR, delta_post, measu_last_).z());*/



         ROS_INFO("error = (%.3f, %.3f, %.3f)", error.x(), error.y(), error.z());
         //滑动均值.
         if(nk >= 0)
         {
           //error covariance.
           err_P_ = vec2Matrix(error, error);

           tf::Matrix3x3 Fk = optimMoveAverageFilter(err_P_);

           ROS_INFO("Fk =(%.6f, %.6f, %.6f\n"
               "             %.6f, %.6f, %.6f\n"
               "             %.6f, %.6f, %.6f)",
                             Fk[0].x(),Fk[0].y(),Fk[0].z(),
                             Fk[1].x(),Fk[1].y(),Fk[1].z(),
                             Fk[2].x(),Fk[2].y(),Fk[2].z());
           tf::Matrix3x3 error_P0 = vec2Matrix(prev_measu0-measu_curr+error, prev_measu0-measu_curr+error);
           tf::Matrix3x3 error_P1 = vec2Matrix(prev_measu1-measu_curr+error, prev_measu1-measu_curr+error);
           tf::Matrix3x3 error_P2 = vec2Matrix(prev_measu2-measu_curr+error, prev_measu2-measu_curr+error);
           tf::Matrix3x3 error_P3 = vec2Matrix(prev_measu3-measu_curr+error, prev_measu3-measu_curr+error);
           tf::Matrix3x3 error_P4 = vec2Matrix(prev_measu4-measu_curr+error, prev_measu4-measu_curr+error);
           tf::Matrix3x3 error_P5 = vec2Matrix(prev_measu5-measu_curr+error, prev_measu5-measu_curr+error);
           tf::Matrix3x3 error_P6 = vec2Matrix(prev_measu6-measu_curr+error, prev_measu6-measu_curr+error);

           tf::Matrix3x3 temp(wc0*error_P0+
                            wc1.x()*error_P1+wc1.y()*error_P2+wc1.z()*error_P3+
                            wc2.x()*error_P4+wc2.y()*error_P4+wc2.z()*error_P6);

    //       R_ = Fk + Py;
           Q_ = K*Fk*K.transpose();
         }
      }
      else
      {
    /*      post_state_.X.setX(QRdata.pose.position.x);
        post_state_.X.setY(QRdata.pose.position.y);
        post_state_.X.setZ(QRdata.pose.position.z);*/
      }
    //    ROS_INFO("post_state = (%.3f, %.3f, %.3f)", post_state_.X.x(), post_state_.X.y(), post_state_.X.z());

      if(debug_)
        out_ekf_ << aukf_pose_.x() << ","
               << aukf_pose_.y() << ","
               << aukf_pose_.z() << ","
               << K.getRow(0).x() << ","
               << K.getRow(1).y() << ","
               << K.getRow(2).z() << ","
               << post_state_.P.getRow(0).x() << ","
               << post_state_.P.getRow(1).y() << ","
               << post_state_.P.getRow(2).z() << ","
               << measu_curr.x() << ","
               << measu_curr.y() << ","
               << post_state_.X.x() << ","
               << post_state_.X.y() << ","
               << post_state_.X.z() << ","
               <<std::endl;

       waitMilli(20);
       last_time_ = current_time_;     //update last time.
  }
}




tf::Matrix3x3 PoseEKF::addMatrix(const tf::Matrix3x3 &m1, const tf::Matrix3x3 &m2)
{
  return tf::Matrix3x3(m1[0].x()+m2[0].x(), m1[0].y()+m2[0].y(), m1[0].z()+m2[0].z(),   //第一行.m[i]为矩阵的行向量.
                       m1[1].x()+m2[1].x(), m1[1].y()+m2[1].y(), m1[1].z()+m2[1].z(),
                       m1[2].x()+m2[2].x(), m1[2].y()+m2[2].y(), m1[2].z()+m2[2].z());
}


tf::Matrix3x3 PoseEKF::reduceMatrix(const tf::Matrix3x3 &m1, const tf::Matrix3x3 &m2)
{
  return tf::Matrix3x3(m1[0].x()-m2[0].x(), m1[0].y()-m2[0].y(), m1[0].z()-m2[0].z(),   //第一行.m[i]为矩阵的行向量.
                       m1[1].x()-m2[1].x(), m1[1].y()-m2[1].y(), m1[1].z()-m2[1].z(),
                       m1[2].x()-m2[2].x(), m1[2].y()-m2[2].y(), m1[2].z()-m2[2].z());
}

//A=L*L'
tf::Matrix3x3 PoseEKF::Cholesky(const tf::Matrix3x3 &m)
{
  float A[3][3] = {0.0};
  float L[3][3] = {0.0};
  for(int raw = 0; raw < 3; raw++)
  {
    A[raw][0] = m[raw].x();
    A[raw][1] = m[raw].y();
    A[raw][2] = m[raw].z();
  }
  const float ztol = 1.0e-5;

  for(int i = 0; i < 3; i++)
  {
    float sum = 0.0;
    for(int k = 0; k < i; k++)
      sum += A[k][i]*A[k][i];
    float d = A[i][i] - sum;
    if (fabs(d) < ztol)
      L[i][i] = 0.0;
    else
    {
      if(d < 0.0)
      {
        ROS_ERROR("Matrix not positive-definite");
        ros::shutdown();
        return tf::Matrix3x3(1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, 1.0);
      }
      L[i][i] = sqrt(d);
      for(int j=i+1; j<3; j++)
      {
        sum = 0.0;
        for(int n=0; n<i; n++)
          sum += A[n][i]*A[n][j];
        if (abs(sum) < ztol)
          sum = 0.0;
        L[i][j] = (A[i][j] - sum) / L[i][i];

      }
    }

  }
  const float l11 = L[0][0];
  const float l21 = L[1][0];
  const float l22 = L[1][1];
  const float l31 = L[2][0];
  const float l32 = L[2][1];
  const float l33 = L[2][2];

  return tf::Matrix3x3(l11, l21, l31,
                       0.0, l22, l32,
                       0.0, 0.0, l33);  // Return L's transpose for ROS TF Matrix with row priority.
}
tf::Vector3 PoseEKF::rootMatrix(const tf::Matrix3x3 &m, int row)
{
  if((row < 0) || (row >3))
  {
    ROS_ERROR("Matrix's index wrong.");
    return tf::Vector3(0.0, 0.0, 0.0);
  }
  tf::Vector3 out(Cholesky(m)[row].x(), Cholesky(m)[row].y(), Cholesky(m)[row].z());
  return tf::Vector3(out.getX(), out.getY(), out.getZ());
}
tf::Vector3 PoseEKF::systemFunc(const tf::Vector3 &v1, const tf::Vector3 &vel, float dt)
{
//  ROS_INFO("v1 = (%.3f, %.3f, %.3f)", v1.x(), v1.y(), v1.z());
  float theta_F = angleTrunc(v1.z() + vel.z()*dt/2.0);
  float f11 = cos(theta_F), f12 = -sin(theta_F), f13 = 0.0;
  float f21 = sin(theta_F), f22 = cos(theta_F), f23 = 0.0;
  float f31 = 0.0, f32 = 0.0, f33 = 1.0;
  tf::Matrix3x3 F(f11, f12, f13,
                  f21, f22, f23,
                  f31, f32, f33);
  tf::Vector3 out((v1 + F * vel * dt).x(), (v1 + F * vel * dt).y(), (v1 + F * vel * dt).z());

  //  ROS_INFO("dt = %.3f", dt);
   // ROS_INFO("vel=(%.3f, %.3f, %.3f)", vel.x(), vel.y(), vel.z());
  //  ROS_INFO("out = (%.3f, %.3f, %.3f)", out.x(), out.y(), out.z());

  return tf::Vector3((v1 + F * vel * dt).x(), (v1 + F * vel * dt).y(), (v1 + F * vel * dt).z());
}
tf::Vector3 updateParticles(const tf::Vector3 &v1, const float movement)
{


}

tf::Vector3 PoseEKF::measuFunc(const tf::Vector3 &QR, const tf::Vector3 &state, const tf::Vector3 &measu)
{
  tf::Vector3 QRpos(QR.x(), QR.y(), 0.0);
  const float QRangle = 0.0;             // 需要计算出当前坐标系与参考坐标系之间的夹角．实际铺设时，所有QR坐标系都是一致的，所以为０．
  T_tag1_tag0.setOrigin(-QRpos);         // (x,y,z)
  T_tag1_tag0.setRotation(tf::createQuaternionFromYaw(-QRangle));

 // tf::Vector3 last_measurement(measu_last_.x(), measu_last_.y(), 0.0);
  tf::Vector3 last_measurement(0.005, 0.005, 0.0);
  const float last_meas_angle = measu_last_.z();
  T_tag0_C0.setOrigin(-last_measurement);
  T_tag0_C0.setRotation(tf::createQuaternionFromYaw(angleTrunc(-last_meas_angle)));

  tf::Vector3 movement(state.x(), state.y(), 0.0);
  const float odom_angle = state.z();
  T_R0_R1.setOrigin(movement);
  T_R0_R1.setRotation(tf::createQuaternionFromYaw(angleTrunc(odom_angle)));

  tf::Transform transformed = T_tag1_tag0 * T_tag0_C0 * T_C0_R0 * T_R0_R1 * T_R1_C1;
 // tf::Transform transformed = T_R1_C1*T_R0_R1*T_C0_R0*T_tag0_C0*T_tag1_tag0;
  tf::Vector3 prev_meas = transformed.getOrigin();
  tf::Matrix3x3 rotation = transformed.getBasis();
  tf::Quaternion q;
  rotation.getRotation(q);

  ROS_DEBUG("******");
  ROS_DEBUG("T_tag1_tag0=([%.3f, %.3f, %.3f, %.3f]", T_tag1_tag0.getBasis()[0].x(), T_tag1_tag0.getBasis()[0].y(), T_tag1_tag0.getBasis()[0].z(), T_tag1_tag0.getOrigin().x());
  ROS_DEBUG("T_tag1_tag0=([%.3f, %.3f, %.3f, %.3f]", T_tag1_tag0.getBasis()[1].x(), T_tag1_tag0.getBasis()[1].y(), T_tag1_tag0.getBasis()[1].z(), T_tag1_tag0.getOrigin().y());
  ROS_DEBUG("T_tag1_tag0=([%.3f, %.3f, %.3f, %.3f]", T_tag1_tag0.getBasis()[2].x(), T_tag1_tag0.getBasis()[2].y(), T_tag1_tag0.getBasis()[2].z(), T_tag1_tag0.getOrigin().z());
  ROS_DEBUG("\n");
  ROS_DEBUG("T_tag0_C0=([%.3f, %.3f, %.3f, %.3f]", T_tag0_C0.getBasis()[0].x(), T_tag0_C0.getBasis()[0].y(), T_tag0_C0.getBasis()[0].z(), T_tag0_C0.getOrigin().x());
  ROS_DEBUG("T_tag0_C0=([%.3f, %.3f, %.3f, %.3f]", T_tag0_C0.getBasis()[1].x(), T_tag0_C0.getBasis()[1].y(), T_tag0_C0.getBasis()[1].z(), T_tag0_C0.getOrigin().y());
  ROS_DEBUG("T_tag0_C0=([%.3f, %.3f, %.3f, %.3f]", T_tag0_C0.getBasis()[2].x(), T_tag0_C0.getBasis()[2].y(), T_tag0_C0.getBasis()[2].z(), T_tag0_C0.getOrigin().z());
  ROS_DEBUG("\n");
  ROS_DEBUG("T_R0_R1=([%.3f, %.3f, %.3f, %.3f]", T_R0_R1.getBasis()[0].x(), T_R0_R1.getBasis()[0].y(), T_R0_R1.getBasis()[0].z(), T_R0_R1.getOrigin().x());
  ROS_DEBUG("T_R0_R1=([%.3f, %.3f, %.3f, %.3f]", T_R0_R1.getBasis()[1].x(), T_R0_R1.getBasis()[1].y(), T_R0_R1.getBasis()[1].z(), T_R0_R1.getOrigin().y());
  ROS_DEBUG("T_R0_R1=([%.3f, %.3f, %.3f, %.3f]", T_R0_R1.getBasis()[2].x(), T_R0_R1.getBasis()[2].y(), T_R0_R1.getBasis()[2].z(), T_R0_R1.getOrigin().z());
  ROS_DEBUG("\n");
  ROS_DEBUG("T_C0_R0=([%.3f, %.3f, %.3f, %.3f]", T_C0_R0.getBasis()[0].x(), T_C0_R0.getBasis()[0].y(), T_C0_R0.getBasis()[0].z(), T_C0_R0.getOrigin().x());
  ROS_DEBUG("T_C0_R0=([%.3f, %.3f, %.3f, %.3f]", T_C0_R0.getBasis()[1].x(), T_C0_R0.getBasis()[1].y(), T_C0_R0.getBasis()[1].z(), T_C0_R0.getOrigin().y());
  ROS_DEBUG("T_C0_R0=([%.3f, %.3f, %.3f, %.3f]", T_C0_R0.getBasis()[2].x(), T_C0_R0.getBasis()[2].y(), T_C0_R0.getBasis()[2].z(), T_C0_R0.getOrigin().z());
  ROS_DEBUG("\n");
  ROS_DEBUG("T_R1_C1=([%.3f, %.3f, %.3f, %.3f]", T_R1_C1.getBasis()[0].x(), T_R1_C1.getBasis()[0].y(), T_R1_C1.getBasis()[0].z(), T_R1_C1.getOrigin().x());
  ROS_DEBUG("T_R1_C1=([%.3f, %.3f, %.3f, %.3f]", T_R1_C1.getBasis()[1].x(), T_R1_C1.getBasis()[1].y(), T_R1_C1.getBasis()[1].z(), T_R1_C1.getOrigin().y());
  ROS_DEBUG("T_R1_C1=([%.3f, %.3f, %.3f, %.3f]", T_R1_C1.getBasis()[2].x(), T_R1_C1.getBasis()[2].y(), T_R1_C1.getBasis()[2].z(), T_R1_C1.getOrigin().z());

  ROS_DEBUG("\n");
  ROS_DEBUG("transformed=([%.3f, %.3f, %.3f, %.3f]", transformed.getBasis()[0].x(), transformed.getBasis()[0].y(), transformed.getBasis()[0].z(), transformed.getOrigin().x());
  ROS_DEBUG("transformed=([%.3f, %.3f, %.3f, %.3f]", transformed.getBasis()[1].x(), transformed.getBasis()[1].y(), transformed.getBasis()[1].z(), transformed.getOrigin().y());
  ROS_DEBUG("transformed=([%.3f, %.3f, %.3f, %.3f]", transformed.getBasis()[2].x(), transformed.getBasis()[2].y(), transformed.getBasis()[2].z(), transformed.getOrigin().z());
  return tf::Vector3(prev_meas.x(), prev_meas.y(), angleTrunc(q.getAngle()));

}
tf::Matrix3x3 PoseEKF::vec2Matrix(const tf::Vector3 &v1, const tf::Vector3 &v2)
{
  return tf::Matrix3x3(v1.x()*v2.x(), v1.x()*v2.y(), v1.x()*v2.z(),
                       v1.y()*v2.x(), v1.y()*v2.y(), v1.y()*v2.z(),
                       v1.z()*v2.x(), v1.z()*v2.y(), v1.z()*v2.z());
}
tf::Matrix3x3 PoseEKF::scale(const tf::Matrix3x3 &m, float k)
{
  return tf::Matrix3x3(k*m[0].x(), k*m[0].y(), k*m[0].z(),
                       k*m[1].x(), k*m[1].y(), k*m[1].z(),
                       k*m[2].x(), k*m[2].y(), k*m[2].z());
}

tf::Matrix3x3 PoseEKF::optimMoveAverageFilter(const tf::Matrix3x3& new_m)
{
  tf::Matrix3x3 mean_sum;               //用于保存均值.
  tf::Matrix3x3 droped_element;     //用于保存被挤出队列的那个元素.
  if(error_buffer_.empty())
  {
      std::cout << "Filter buffer is empty!!"<<std::endl;
      return tf::Matrix3x3::getIdentity();
  }
  static tf::Matrix3x3 sum(0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0);

  if(error_buffer_.size() >= error_buffer_size_)
  {
    droped_element = error_buffer_.front();       //为什么一直是0.构造函数会默认实现operator=,不需要手动实现.
    error_buffer_.pop();
    error_buffer_.push(new_m);
  }

  if((short int)error_buffer_.size() == error_buffer_size_)       //确保写入元素成功.
  {
    //TODO:计算平均值.
    //求新的和
    sum = sum + new_m - droped_element;
    //求平均值.
    mean_sum = sum*(1.0/(float)error_buffer_size_);
  }
  else
  {
    std::cout << "Filter write is Wrong!!"<<std::endl;
    return tf::Matrix3x3::getIdentity();
  }
  return tf::Matrix3x3(mean_sum[0].x(), mean_sum[0].y(), mean_sum[0].z(),
                       mean_sum[1].x(), mean_sum[1].y(), mean_sum[1].z(),
                       mean_sum[2].x(), mean_sum[2].y(), mean_sum[2].z());
}

void PoseEKF::setTransMatrix()
{
  T_C0_R0.setOrigin(C2R);
  T_C0_R0.setRotation(tf::createQuaternionFromYaw(Theta1));
  T_R1_C1.setOrigin(R2C);
  T_R1_C1.setRotation(tf::createQuaternionFromYaw(Theta3));
}


void PoseEKF::waitMilli(int milliseconds)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds(milliseconds));
}

float angleTrunc(float angle)
{
  while (angle<0.0)
      angle += M_PI;
  return (fmodf(angle + M_PI_2, M_PI) - M_PI_2);
}
