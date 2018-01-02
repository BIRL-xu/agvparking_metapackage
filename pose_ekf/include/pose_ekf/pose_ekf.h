/*
 * pose_ekf.h
 *
 *  Created on: Apr 11, 2017
 *      Author: paul
 */

#ifndef INCLUDE_POSE_EKF_H_
#define INCLUDE_POSE_EKF_H_

#include "ros/ros.h"
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "agvparking_msg/AgvOdom.h"
#include <fstream>      //读写文件流的头文件．
#include <iostream>
#include <queue>

#define SIGN(x) ((x > 0) - (x < 0))
float angleTrunc(float angle);
typedef struct EKFElements
{
  tf::Matrix3x3 P;      //协方差矩阵.
  tf::Vector3 X;        //系统估计状态.
  EKFElements():X(0.0, 0.0, 0.0)
  {
    P.setValue(0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0);
  }
}EKFELEMENTS;
class PoseEKF
{
public:
  PoseEKF();
  ~PoseEKF();

private:
  //topics callback.
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void QRCallback(const agvparking_msg::AgvOdom::ConstPtr &QR_info);
  void update();
  void UKFupdate();
  void UKFupdateTest();
  //论文实验
  void UKFUpdateOdom();         //standard UKF
  void AUKFUpdateOdom();        // Adaptive UKF.
  void EKFUpdateOdom();
  void UKFAdjust();

  void waitMilli(int milliseconds);
  tf::Matrix3x3 addMatrix(const tf::Matrix3x3 &m1, const tf::Matrix3x3 &m2);
  tf::Matrix3x3 reduceMatrix(const tf::Matrix3x3 &m1, const tf::Matrix3x3 &m2);
  //Cholesky decomposition.
  tf::Matrix3x3 Cholesky(const tf::Matrix3x3 &m);
  tf::Vector3 rootMatrix(const tf::Matrix3x3 &m, int row);
  tf::Vector3 systemFunc(const tf::Vector3 &v1, const tf::Vector3 &vel, float dt);
  tf::Vector3 measuFunc(const tf::Vector3 &dQR, const tf::Vector3 &state, const tf::Vector3 &measu);
  tf::Matrix3x3 vec2Matrix(const tf::Vector3 &v1, const tf::Vector3 &v2);
  tf::Matrix3x3 scale(const tf::Matrix3x3 &m, float k);
  tf::Matrix3x3 optimMoveAverageFilter(const tf::Matrix3x3& new_m);
  tf::Vector3 updateParticles(const tf::Vector3 &v1, const float movement);
  void setTransMatrix();
private:
  //variables
  EKFELEMENTS prev_state_, post_state_;      //估计前后的状态.
  tf::Matrix3x3 Q_ ;            //系统高斯白噪声.
  tf::Matrix3x3 R_;             //观测白噪声.
  tf::Matrix3x3 err_P_;
  tf::Vector3 QRcoor_;          //QR坐标系.
  tf::Vector3 measu_last_;
  tf::Vector3 ekf_pose_, ukf_pose_, aukf_pose_, raw_odom_;            //里程计计算的位姿.
  tf::Vector3 error_;
  // Homogeneous Matrix
  tf::Transform T_R1_C1, T_R0_R1, T_C0_R0, T_tag0_C0, T_tag1_tag0;
  bool initialized_;
  bool debug_;

  double t_last_;
  double factor_;
  double disp_odom_;    //修正前的里程计变化量.
  tf::Vector3 ref_;
  std::ofstream out_ekf_, out_error_, out_error_x_, out_error_y_, out_pose_error_;

  std::queue<tf::Matrix3x3> error_buffer_;        //用于保存残差.
  unsigned int error_buffer_size_;
  //ros
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, QR_sub_;
  agvparking_msg::AgvOdom QRinfo_;
  nav_msgs::Odometry odom_;
  ros::Time current_time_, last_time_;
  //thread
  boost::timed_mutex QRmutex_, odom_mutex_;

};


inline tf::Matrix3x3 operator+(const tf::Matrix3x3 &m1, const tf::Matrix3x3 &m2)
{

  return tf::Matrix3x3(m1[0].x()+m2[0].x(), m1[0].y()+m2[0].y(), m1[0].z()+m2[0].z(),   //第一行.m[i]为矩阵的行向量.
                       m1[1].x()+m2[1].x(), m1[1].y()+m2[1].y(), m1[1].z()+m2[1].z(),
                       m1[2].x()+m2[2].x(), m1[2].y()+m2[2].y(), m1[2].z()+m2[2].z());
}
inline tf::Matrix3x3 operator-(const tf::Matrix3x3 &m1, const tf::Matrix3x3 &m2)
{
  return tf::Matrix3x3(m1[0].x()-m2[0].x(), m1[0].y()-m2[0].y(), m1[0].z()-m2[0].z(),   //第一行.m[i]为矩阵的行向量.
                       m1[1].x()-m2[1].x(), m1[1].y()-m2[1].y(), m1[1].z()-m2[1].z(),
                       m1[2].x()-m2[2].x(), m1[2].y()-m2[2].y(), m1[2].z()-m2[2].z());
}
inline tf::Matrix3x3 operator*(const tfScalar &s, const tf::Matrix3x3 &m)
{
 // tf::Vector3 factor(s, s, s);

  return tf::Matrix3x3(s*m[0].x(), s*m[0].y(), s*m[0].z(),
                    s*m[1].x(), s*m[1].y(), s*m[1].z(),
                    s*m[2].x(), s*m[2].y(), s*m[2].z());
}
inline tf::Matrix3x3 operator*(const tf::Matrix3x3 &m, const tfScalar &s)
{
  tf::Vector3 factor(s, s, s);
  return m.scaled(factor);
}


#endif /* INCLUDE_POSE_EKF_H_ */
