/*
 * tcp_driver.h
 *
 *  Created on: Dec 9, 2016
 *      Author: paul
 */

#ifndef INCLUDE_TCP_DRIVER_TCP_DRIVER_H_
#define INCLUDE_TCP_DRIVER_TCP_DRIVER_H_
#include "ros/ros.h"
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <queue>

#define SIGN(x) ((x > 0) - (x < 0))     //取符号函数.

//tolerance
#define POSITION_TOLERANCE 0.005        //m.
#define ANGLE_TOLERANCE 0.05             //rad.
#define ALIGN_VEL 0.02                  //m/s.
#define QRDISTANCE      0.5     //QR码的间距，默认50cm.
typedef unsigned char Byte;

using namespace boost::asio::ip;
void waitMilli(int milliseconds);

class ServerSocket
{
public:
  ServerSocket(boost::asio::io_service &io_sev, const int port=3000);
  ~ServerSocket();

private:
  void listenOnPort();
  void socketWrite();
  void socketRead();
  void velCallback(const geometry_msgs::Twist::ConstPtr &msg_ptr);
  inline void dataHandle();
  inline void readDataHandle();
  void odom();
  void rawOdom();

  float angleMapAndRad(const unsigned short int &angle);
  typedef struct IntCMDVEL
  {
    short int vx;
    short int vy;
    short int vw;
    short int vz;
/*    IntCMDVEL &operator=(const IntCMDVEL &cpy)        //该重载操作符右编译器自动实现.
    {
      if(this != &cpy)
      {
        this->vx = cpy.vx;
        this->vy = cpy.vy;
        this->vw = cpy.vw;
        this->vz = cpy.vz;
      }
      return *this;
    }*/
    IntCMDVEL()
    {
      vx = 0;
      vy = 0;
      vw = 0;
      vz = 0;
    }
  }IntCmdVel;
  IntCmdVel optimMoveAverageFilter(const IntCmdVel& cmd_vel);

private:
  union DataExchange
  {
    short int int_num;
    Byte data;
  }dataEx[4];

  struct Pose
  {
    float x;
    float y;
    float theta;
    Pose()
    {
      x = 0.0;
      y = 0.0;
      theta = 0.0;
    }
  }pose_;

  struct QRInfo
  {
    short int x;
    char y;
    unsigned short int angle;
    unsigned short int num;

    QRInfo()
    {
      x=0,y=0;
      angle =0;
      num = 0;
    }
  }QR_info_;

  struct FeedbackData
  {
    float x;    //m.
    float y;    //m.
    float angle;        //rad.
    float vel[3];       //feedback_vel.
    unsigned short int num;
    FeedbackData()
    {
      x = 0.0;
      y = 0.0;
      angle = 0.0;
      num = 0;
      vel[0] = 0.0;
      vel[1] = 0.0;
      vel[2] = 0.0;
    }

  }recv_data_;

  //计算距离
  float computePoseDist(const Pose &pose1, const Pose &pose2);
  float computeDist(const float &k1, const float &k2);
private:
  //socket.
  tcp::socket server_socket_;
  tcp::acceptor acceptor_;
  //thread.
  boost::shared_ptr<boost::thread> listen_thrd_ptr_;
  boost::shared_ptr<boost::thread> send_thrd_ptr_;
  boost::shared_ptr<boost::thread> read_thrd_ptr_, read_handle_ptr_, odom_thrd_;
  boost::shared_ptr<boost::thread> align_ptr_;

  boost::mutex cmdVel_mutex_, feedbackVel_mutex_;       //速度数据转换期间的互斥体。
  boost::mutex read_mutex_;     //TCP接收数据锁存。
  boost::mutex publish_mutex_;
  //variable
  bool tcp_alive_;     //监听状态。
  bool getQR_;
  float vel4f_[4];
  Byte sample_buff_[4];
  Byte read_buff_[18];
  std::queue<IntCmdVel> cmd_buffer_;         //滑动均值滤波的缓存器.
  IntCmdVel last_cmd_;
  const short int cmd_buf_size_;
  //odom scale.
  double odom_scale_x_;
  double odom_scale_y_;
  double odom_scale_yaw_;
  bool odom_reset_, align_first_, aligned_;
  //speed limits.
  float limit_vx_;
  float limit_vy_;
  float limit_vw_;
  Pose last_pose_;//识别到上一个二维码时的位置。

  //ros
  ros::NodeHandle nh_;
  ros::Subscriber vel_sub_;
  ros::Publisher vel_pub_, QR_odom_pub_ ;
  ros::Time current_time_, last_time_;

  geometry_msgs::TransformStamped odom_trans_;
  tf::TransformBroadcaster odom_broadcaster_;
  nav_msgs::Odometry odom_msg_;
  ros::Publisher odom_pub_;
};




#endif /* INCLUDE_TCP_DRIVER_TCP_DRIVER_H_ */
