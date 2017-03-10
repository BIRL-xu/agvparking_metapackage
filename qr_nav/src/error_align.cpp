/*
 * error_align.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: paul
 */
#include "qr_nav/error_align.h"

PoseAlign::PoseAlign(const Target goal[], long unsigned int size):nh_(ros::NodeHandle("~")),aligning_(false),aligned_(false),x_aligning_(false),y_aligning_(false),yaw_aligning_(false),navigation_(false)
,goback_(false)
{
  memset(pose3f_ ,0 , 3);
  memset(vel3f_ ,0 , 3);
  moving_ = false;
  time_update_ = false;
  traj_aligning_ = false;
  InitErr_.x = 0.0;
  InitErr_.y = 0.0;
  InitErr_.yaw = 0.0;
  //goal initialize.
  setGoals(goal, size);
  goals_num_ = size;

  d_const_ = 0.0;
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.angular.z = 0.0;
  position_tolerance_ = 0.01;   //8mm.
  angle_tolerance_ = 0.05;        //0.05rad=2.87°.
  odom_ref_ = 0.0;
  nh_.param("navigation", navigation_);
//  now_time_ = ros::Time::now();
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 5);    //"/cmd_vel"为全局名称，"cmd_vel"为局部名称，发布时自动加上节点名空间，如/qr_nav_node/cmd_vel
  QR_odom_sub_ = nh_.subscribe<agvparking_msg::AgvOdom>("/qr_odom", 1, &PoseAlign::qrInfoCallback, this);

  //boost thread.
  boost::shared_ptr<boost::thread> align_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseAlign::getPoseError, this)));
 // boost::shared_ptr<boost::thread> move_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseAlign::alignAndMove, this)));
  boost::shared_ptr<boost::thread> move_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PoseAlign::move, this)));
}

PoseAlign::~PoseAlign()
{

}

void PoseAlign::setGoals(const Target goal[], const short int size)
{
  for(int i = 0; i < size; i++)
  {
    goal_[i].x = goal[i].x;
    goal_[i].y = goal[i].y;
  }
}
void PoseAlign::qrInfoCallback(const agvparking_msg::AgvOdom::ConstPtr &qr_info)
{
  boost::unique_lock<boost::shared_mutex> lock(odom_mutex_);
  //odom pose
  qr_odom_.odom.x = qr_info->pose.position.x;
  qr_odom_.odom.y = qr_info->pose.position.y;
  qr_odom_.odom.yaw =  tf::getYaw(qr_info->pose.orientation);  //里程计航向角。
  //odom velocity.
  qr_odom_.odom.vx = qr_info->twist.linear.x;
  qr_odom_.odom.vy = qr_info->twist.linear.y;
  qr_odom_.odom.vw = qr_info->twist.angular.z;

  qr_odom_.pose.tag_num = (unsigned short int)qr_info->qr.Tagnum;
  qr_odom_.pose.x = qr_info->qr.x/1000.0;         //m.
  qr_odom_.pose.y = qr_info->qr.y/1000.0;         //m.
  qr_odom_.pose.angle = qr_info->qr.angle;        //rad.
//  std::cout << "error_x=" << qr_odom_.pose.x<< std::endl;
//  std::cout << "error_y=" << qr_odom_.pose.y<< std::endl;
//  std::cout << "error_angle=" << qr_odom_.pose.angle<< std::endl;
//    std::cout << "tag_num=" << qr_pose_.tag_num << std::endl;
  lock.unlock();
  waitMilli(10);
}

void PoseAlign::getPoseError()
{
  while(true)
  {

    float odom_ref = 0.0;
    {
 //     boost::unique_lock<boost::mutex> lock(odom_ref_mutex_);

      odom_ref = odom_ref_;
//      cond_.notify_all();
//      cond_.wait(odom_ref_mutex_);
    }
    OdomData odom_data;
    {
      boost::shared_lock<boost::shared_mutex> lock(odom_mutex_);
      qr_pose_ = qr_odom_.pose;
      odom_data = qr_odom_.odom;        //赋值给新变量，以尽快释放占用的线程锁。
    }
    boost::unique_lock<boost::timed_mutex> error_lock(error_mutex_);
    boost::unique_lock<boost::timed_mutex> state_lock(state_mutex_);
    if(qr_pose_.tag_num)                //识别到二维码时，以二维码的数据作为调整依据。
    {
      if((fabs(qr_pose_.x) < position_tolerance_) && (fabs(qr_pose_.y) < position_tolerance_) && fabs(qr_pose_.angle) < angle_tolerance_)
      {
        aligning_ = false;
        aligned_ = true;
        x_aligning_ = false;
        y_aligning_ = false;
        yaw_aligning_ = false;

        traj_aligning_ = false;
        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();
        //error.
        err_.x = 0.0;
        err_.y = 0.0;
        err_.yaw = 0.0;

        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();
      }
      else
      {
        aligned_ = false;
        aligning_ = true;
       if(fabs(qr_pose_.x) >= position_tolerance_)
       {
         x_aligning_ = true;
         err_.x = qr_pose_.x;
         d_const_ = odom_data.y + PREDISTANCE;
       }
       else
       {
         err_.x = 0.0;
         x_aligning_ = false;
       }
       if(fabs(qr_pose_.y) >= position_tolerance_)
       {
         err_.y = qr_pose_.y;
         y_aligning_ = true;
         d_const_ = odom_data.x + PREDISTANCE;
       }
       else
       {
         err_.y = 0.0;
         y_aligning_ = false;
       }
       if(fabs(qr_pose_.angle) >= angle_tolerance_)
       {
         err_.yaw = qr_pose_.angle;
         yaw_aligning_ = true;
       }
       else
       {
         err_.yaw = 0.0;
         yaw_aligning_ = false;
       }
       boost::timed_mutex *sta_ptr = state_lock.release();
       sta_ptr->unlock();

       boost::timed_mutex *err_ptr = error_lock.release();
       err_ptr->unlock();
      }


/*

      if((fabs(qr_pose_.x) >= position_tolerance_) ||
          (fabs(qr_pose_.y) >= position_tolerance_) ||
          (fabs(qr_pose_.angle) >= angle_tolerance_))
      {
        err_.x = qr_pose_.x;
        err_.y = qr_pose_.y;
        err_.yaw = qr_pose_.angle;

        aligning_ = true;                       //需要调整。

        if(fabs(qr_pose_.x) >= position_tolerance_)
          x_aligning_ = true;
        else
          x_aligning_ = false;
        if(fabs(qr_pose_.y) >= position_tolerance_)
          y_aligning_ = true;
        else
          y_aligning_ = false;
        if(fabs(qr_pose_.angle) >= angle_tolerance_)
          yaw_aligning_ = true;
        else
          yaw_aligning_ = false;
        lock.unlock();
        std::cout << "aligning_=" << aligning_<< std::endl;
        std::cout << "aligning_x_=" << x_aligning_<< std::endl;
        std::cout << "aligning_y_=" << y_aligning_<< std::endl;
        std::cout << "aligning_yaw_=" << yaw_aligning_<< std::endl;
      }
      else
      {
        err_.x = 0.0;
        err_.y = 0.0;
        err_.yaw = 0.0;
        aligning_ = false;
        x_aligning_ = false;
        y_aligning_ = false;
        yaw_aligning_ = false;
        aligned_ = true;       //调整完毕。
        lock.unlock();
      }
*/

    }
    else                      //没有识别到二维码时，以里程计的数据作为调整依据。所以，必须校正里程计以保证准确性。
    {
      if((motion_state_.expeted_dir == "Y+"))
      {
        err_.x = odom_data.x - odom_ref;
        err_.y = 0.0;
        err_.yaw = odom_data.yaw;
        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();

        y_aligning_ = false;
        if(fabs(fabs(odom_data.x)-fabs(odom_ref)) >= TRAVEL_ERROR)
          x_aligning_ = true;
        else
          x_aligning_ = false;

        if(fabs(odom_data.yaw) >= angle_tolerance_)
          yaw_aligning_ = true;
        else
          yaw_aligning_ = false;

        if(x_aligning_ || y_aligning_ || yaw_aligning_)
        {
          aligning_ = true;
          aligned_ = false;
        }
        else if(!x_aligning_ && !y_aligning_ && !yaw_aligning_)
        {
          aligning_ = false;
          aligned_ = true;
          traj_aligning_ = false;
        }
        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();


        d_const_ = odom_data.y + PREDISTANCE;
      }
      else if((motion_state_.expeted_dir == "X+"))
      {
        err_.x = 0.0;
        err_.y = odom_data.y - odom_ref;
        err_.yaw = odom_data.yaw;
/*        printf("odom_y = %.3f\n", odom_data.y);
        printf("odom_ref = %.3f\n", odom_ref);
        printf("err_y = %.3f\n", err_.y);
        printf("err_yaw = %.3f\n", err_.yaw);*/
        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();

        x_aligning_ = false;
        if(fabs(fabs(odom_data.y)-fabs(odom_ref)) >= TRAVEL_ERROR)
        {
          y_aligning_ = true;
        }
        else
        {
          y_aligning_ = false;
        }
        d_const_ = odom_data.x + PREDISTANCE;
        if(fabs(odom_data.yaw) >= angle_tolerance_)
          yaw_aligning_ = true;
        else
          yaw_aligning_ = false;

        if(x_aligning_ || y_aligning_ || yaw_aligning_)
        {
          aligning_ = true;
          aligned_ = false;
        }
        else if(!x_aligning_ && !y_aligning_ && !yaw_aligning_)
        {
          aligning_ = false;
          aligned_ = true;
          traj_aligning_ = false;
        }
        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();

      }
      else if((motion_state_.expeted_dir == "Y-"))
      {
        err_.x = odom_data.x - odom_ref;
        err_.y = 0.0;
        err_.yaw = odom_data.yaw;
        error_lock.unlock();
        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();

        y_aligning_ = false;
        if(fabs(fabs(odom_data.x)-fabs(odom_ref)) >= TRAVEL_ERROR)
        {
          x_aligning_ = true;
          d_const_ = odom_data.y + PREDISTANCE;
        }
        else
        {
          x_aligning_ = false;
        }
        if(fabs(odom_data.yaw) >= angle_tolerance_)
          yaw_aligning_ = true;
        else
          yaw_aligning_ = false;

        if(x_aligning_ || y_aligning_ || yaw_aligning_)
        {
          aligning_ = true;
          aligned_ = false;
        }
        else if(!x_aligning_ && !y_aligning_ && !yaw_aligning_)
        {
          aligning_ = false;
          aligned_ = true;
          traj_aligning_ = false;
        }
        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();

      }
      else if((motion_state_.expeted_dir == "X-"))
      {

        err_.x = 0.0;
        err_.y = odom_data.y - odom_ref;
        err_.yaw = odom_data.yaw;
        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();

        printf("odom_y = %.3f\n", odom_data.y);
         printf("odom_ref = %.3f\n", odom_ref);
         printf("err_y = ========%.3f\n", err_.y);
         x_aligning_ = false;
        if(fabs(fabs(odom_data.y)-fabs(odom_ref)) >= TRAVEL_ERROR)
        {
          y_aligning_ = true;
          d_const_ = odom_data.x + PREDISTANCE;
        }
        else
        {
          y_aligning_ = false;
        }
        if(fabs(odom_data.yaw) >= angle_tolerance_)
          yaw_aligning_ = true;
        else
          yaw_aligning_ = false;

        if(x_aligning_ || y_aligning_ || yaw_aligning_)
        {
          aligning_ = true;
          aligned_ = false;
        }
        else if(!x_aligning_ && !y_aligning_ && !yaw_aligning_)
        {
          aligning_ = false;
          aligned_ = true;
          traj_aligning_ = false;
        }

        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();

      }
/*      else if(fabs(odom_data.yaw) >= angle_tolerance_)//只需要调整yaw.
      {

        aligning_ = true;
        x_aligning_ = false;
        y_aligning_ = false;
        yaw_aligning_ = true;
        aligned_ = false;
        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();

        err_.x = 0.0;
        err_.y = 0.0;
        err_.yaw = 0.0;
        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();

      }*/
      else
      {
        aligning_ = false;
        x_aligning_ = false;
        y_aligning_ = false;
        yaw_aligning_ = false;
        aligned_ = true;
        traj_aligning_ = false;
        boost::timed_mutex *sta_ptr = state_lock.release();
        sta_ptr->unlock();

        err_.x = 0.0;
        err_.y = 0.0;
        err_.yaw = 0.0;
        boost::timed_mutex *err_ptr = error_lock.release();
        err_ptr->unlock();
      }
    }
    waitMilli(10);
  }
}

void PoseAlign::alignAndMove()
{
  while(true)
  {
    geometry_msgs::Twist cmd_vel;
    nh_.param("navigation", this->navigation_, false);
///    waitMilli(50);
    unsigned short int align_time = 50;
    float vx = 0.0, vy = 0.0, vw = 0.0;
    QROdom odom;
    {
      boost::shared_lock<boost::shared_mutex> lock(odom_mutex_);
      odom = qr_odom_;
    }

/*
      //print states.
      printf("motion states-------------\n");
      printf("moving_dir=%s\n",motion_state_.moving_dir.c_str());
      printf("expected_dir=%s\n",motion_state_.expeted_dir.c_str());
      printf("speedup=%d\n",motion_state_.speedup);
      printf("start=%d\n",motion_state_.start);
      printf("\n");
*/

/*
      //print error
      printf("motion error--------------\n");
      printf("error=(%.3f, %.3f, %.3f)\n", err_.x, err_.y, err_.yaw);
      printf("\n");

      //print bool variables
      printf("Align variables-----------\n");
      printf("aligning=%s\n", aligning_==false?"False":"True");
      printf("aligned=%s\n", aligned_==false?"False":"True");
      printf("x_aligning=%s\n", x_aligning_==false?"False":"True");
      printf("y_aligning=%s\n", y_aligning_==false?"False":"True");
      printf("yaw_aligning=%s\n", yaw_aligning_==false?"False":"True");
      printf("moving=%s\n", moving_==false?"False":"True");
      printf("\n");
*/

//    printf("odom_ref = %.3f\n", odom_ref_);

    if(!navigation_)
      continue;

    Error error;
    bool aligning, x_aligning, y_aligning, yaw_aligning, aligned;
    {
      boost::unique_lock<boost::timed_mutex> err_lock(error_mutex_, boost::try_to_lock);
      if(err_lock.owns_lock() || err_lock.try_lock_for(boost::chrono::milliseconds(10)))
      {
        error = err_;
        err_lock.unlock();
      }
      boost::unique_lock<boost::timed_mutex> state_lock(state_mutex_, boost::try_to_lock);
      if(state_lock.owns_lock() || state_lock.try_lock_for(boost::chrono::milliseconds(10)))
      {
        aligning = aligning_;
        x_aligning = x_aligning_;
        y_aligning = y_aligning_;
        yaw_aligning = yaw_aligning_;
        aligned = aligned_;
        state_lock.unlock();
      }
/*
      printf("aligning=%s\n", aligning==false?"False":"True");
      printf("aligned=%s\n", aligned==false?"False":"True");
      printf("x_aligning=%s\n", x_aligning==false?"False":"True");
      printf("y_aligning=%s\n", y_aligning==false?"False":"True");
      printf("yaw_aligning=%s\n", yaw_aligning==false?"False":"True");
*/
    }
//-------------------------------
    //for debug
/*    {
      aligning = false;
      odom.pose.tag_num = 1;
      motion_state_.start = 1;
    }*/
//-------------------------------


    if(!aligning)  //不需要调整。
    {
 //   std::cout << "no aligning" << std::endl;
      switch (odom.pose.tag_num)
      {
        case 0:
          cmd_vel_ = last_cmd_;
          break;
        case 1:
          if(motion_state_.start == 1)
          {
            static ros::WallTime lift_time = ros::WallTime::now();    //计时起始处需要设定。
            if(time_update_)
            {
              lift_time = ros::WallTime::now(); //跟新计时起点。
              time_update_ = false;
            }
            last_time_ = ros::WallTime::now();
            unsigned int dt = ((last_time_.sec*1000+last_time_.nsec/1000000) - (lift_time.sec*1000+lift_time.nsec/1000000));       //s*1000+ns/1000000=ms.
//            printf("dt = %d\n", dt);       //1ms = 1000x1000ns.
            if((dt >= 5000) && !moving_)
            {
        //    printf("moving\n");
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = START_VEL;
              cmd_vel_.linear.z = 0.0;
              cmd_vel_.angular.z = 0.0;
              //update state.
              motion_state_.moving_dir = "";        //待识别到2号码时更新。
              motion_state_.expeted_dir = "Y+";
              motion_state_.speedup = 1;
         //     motion_state_.start = 0;
              moving_ = true;
              goback_ = false;
            }
            else if(!moving_)
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 1.0;      //抬升。
              cmd_vel_.angular.z = 0.0;
     //       std::cout << "lifting" << std::endl;
            }

          }
          else if(motion_state_.start == 2)     //作为终点。
          {
            static ros::WallTime destination_time = ros::WallTime::now();    //计时起始处需要设定。
            if(time_update_)
            {
              destination_time = ros::WallTime::now();
              time_update_ = false;
            }
            last_time_ = ros::WallTime::now();
            unsigned int dt = ((last_time_.sec*1000 + last_time_.nsec/1000000) - (destination_time.sec*1000 + destination_time.nsec/1000000));       //s*1000+ns/1000000=ms.

            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = -1.0;      //收回。
            cmd_vel_.angular.z = 0.0;
            //update state.
            motion_state_.moving_dir = "";
            motion_state_.expeted_dir = "";
            motion_state_.speedup = 0;
            moving_ = false;
            if(dt >= 5000)
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;      //收回。
              cmd_vel_.angular.z = 0.0;
              motion_state_.start = 1;
              time_update_ = true;
            }
          }
          else  //异常情况，停止。
          {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.z = 0.0;
          }
          break;
        case 2:
          //经过2号码后改变加减速状态。
/*              if(motion_state_.speedup == 1)
            motion_state_.speedup = -1;
          else if(motion_state_.speedup == -1)
            motion_state_.speedup = 1;*/

          if((motion_state_.expeted_dir == "X+") && (motion_state_.moving_dir != "X+"))
          {
            motion_state_.moving_dir = "X+";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }
          else if(motion_state_.expeted_dir == "Y+" && (motion_state_.moving_dir != "Y+"))
          {
            motion_state_.moving_dir = "Y+";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }
          else if(motion_state_.expeted_dir == "X-" && (motion_state_.moving_dir != "X-"))
          {
            motion_state_.moving_dir = "X-";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }

          else if(motion_state_.expeted_dir == "Y-" && (motion_state_.moving_dir != "Y-"))
          {
            motion_state_.moving_dir = "Y-";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }
          break;
        case 3:
          bool motion;
          static  ros::WallTime stop_time = ros::WallTime::now();
          if(time_update_)
          {
            stop_time = ros::WallTime::now();
            time_update_ = false;
          }
          last_time_ = ros::WallTime::now();
          unsigned int dt_stop;
          dt_stop = ((last_time_.sec*1000 + last_time_.nsec/1000000) - (stop_time.sec*1000 + stop_time.nsec/1000000));       //s*1000+ns/1000000=ms.

          if(dt_stop >= 2500)
            motion = true;
          else
            motion = false;

          if(motion_state_.moving_dir == "Y+")
          {
            if(motion)
            {
              motion_state_.speedup = 1;
              //多处写入，使用条件互斥量较好。
 //             boost::unique_lock<boost::mutex> lock(odom_ref_mutex_);
              odom_ref_ = odom.odom.y;        //沿X方向运动时，里程计y作为调整参考。
 //             cond_.notify_all();
 //             cond_.wait(odom_ref_mutex_);
              if(!goback_)
              {
                motion_state_.expeted_dir = "X+";
                cmd_vel_.linear.x = START_VEL;
              }
              else
              {
                motion_state_.expeted_dir = "X-";
                cmd_vel_.linear.x = -START_VEL;
              }
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;
              cmd_vel_.angular.z = 0.0;
              moving_ = true;
            }
            else        //时间不到时，保持停止状态。
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
  //          moving_ = false;
            }

          }
          else if(motion_state_.moving_dir == "Y-")
          {
            if(motion)
            {
              motion_state_.expeted_dir = "X-";
              motion_state_.speedup = 1;

 //             boost::unique_lock<boost::mutex> lock(odom_ref_mutex_);
 //             cond_.wait(odom_ref_mutex_);
              odom_ref_ = odom.odom.y;       //沿X方向运动时，里程计y作为调整参考。
 //             cond_.notify_all();

              cmd_vel_.linear.x = -START_VEL;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
              moving_ = true;
  //            motion = false;
            }
            else         //时间不到时，保持停止状态。
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
   //           moving_ = false;
            }
          }
          else if(motion_state_.moving_dir == "X+")
          {
            if(motion)
            {
              motion_state_.expeted_dir = "Y-";
       //       motion_state_.moving_dir = "Y-";
              motion_state_.speedup = 1;

 //             boost::unique_lock<boost::mutex> lock(odom_ref_mutex_);
 //             cond_.wait(odom_ref_mutex_);
              odom_ref_ = odom.odom.x;       //沿X方向运动时，里程计y作为调整参考。
 //             cond_.notify_all();

              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = -START_VEL;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
              moving_ = true;
            }
           else
           {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
     //         moving_ = false;
            }
          }
          else if(motion_state_.moving_dir == "X-")
          {
            if(motion)
            {
              motion_state_.expeted_dir = "Y-";
        //      motion_state_.moving_dir = "Y-";
              motion_state_.speedup = 1;

  //            boost::unique_lock<boost::mutex> lock(odom_ref_mutex_);
 //             cond_.wait(odom_ref_mutex_);
              odom_ref_ = odom.odom.x;       //沿X方向运动时，里程计y作为调整参考。
 //             cond_.notify_all();

              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = -START_VEL;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
              moving_ = true;
            }
           else
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;      //先不抬升。
              cmd_vel_.angular.z = 0.0;
      //        moving_ = false;
            }
          }
          break;
        case 4:
          if(motion_state_.speedup == 1)    //加速
          {
          //  motion_state_.speedup = -1;
            if(motion_state_.expeted_dir == "X+")
            {
              cmd_vel_.linear.x = TRAVEL_VEL_X;
              cmd_vel_.linear.y = 0.0;
            }
            else if(motion_state_.expeted_dir == "X-")
            {
              cmd_vel_.linear.x = -TRAVEL_VEL_X;
              cmd_vel_.linear.y = 0.0;
            }
            else if(motion_state_.expeted_dir == "Y+")
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = TRAVEL_VEL_X;
            }
            else if(motion_state_.expeted_dir == "Y-")
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = -TRAVEL_VEL_Y;
            }
          }
          else if(motion_state_.speedup == -1)      //减速。
          {
            moving_ = false;
            time_update_ = true;
          //  motion_state_.speedup = 1;
            if(motion_state_.expeted_dir == "X+")
            {
              cmd_vel_.linear.x = START_VEL;
              cmd_vel_.linear.y = 0.0;
            }
            else if(motion_state_.expeted_dir == "X-")
            {
              cmd_vel_.linear.x = -START_VEL;
              cmd_vel_.linear.y = 0.0;
            }
            else if(motion_state_.expeted_dir == "Y+")
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = START_VEL;
            }
            else if(motion_state_.expeted_dir == "Y-")
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = START_VEL;
              motion_state_.start = 2;      //设置终点状态。
            }
          }
          else
          {
            //do nothing.
          }
          break;
        case 5:
          if(motion_state_.start == 2)      //作为终点
          {
            static ros::WallTime park_time = ros::WallTime::now();    //计时起始处需要设定。
            if(time_update_)
            {
              park_time = ros::WallTime::now();
              time_update_ = false;
            }
            last_time_ = ros::WallTime::now();
            unsigned int dt = ((last_time_.sec*1000 + last_time_.nsec/1000000) - (park_time.sec*1000 + park_time.nsec/1000000));       //s*1000+ns/1000000=ms.

            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = -1.0;        //收回支撑。
            cmd_vel_.angular.z = 0.0;
            //update state.
            motion_state_.moving_dir = "";
            motion_state_.expeted_dir = "";
            motion_state_.speedup = 0;
         // motion_state_.start = 0;
            moving_ = false;

            if(dt >= 5000)
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 0.0;        //收回支撑。
              cmd_vel_.angular.z = 0.0;
              motion_state_.start = 1;      //改变为起点。
              time_update_ = true;
            }
          }
          else if(motion_state_.start == 1)
          {
            static ros::WallTime lift5_time = ros::WallTime::now();    //计时起始处需要设定。
            if(time_update_)
            {
              lift5_time = ros::WallTime::now();
              time_update_ = false;
            }
            last_time_ = ros::WallTime::now();
            unsigned int dt = ((last_time_.sec*1000 + last_time_.nsec/1000000) - (lift5_time.sec*1000 + lift5_time.nsec/1000000));       //s*1000+ns/1000000=ms.

            if((dt >= 5000) && !moving_)
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = START_VEL;
              cmd_vel_.linear.z = 0.0;
              cmd_vel_.angular.z = 0.0;
              //update state.
              motion_state_.moving_dir = "";
              motion_state_.expeted_dir = "Y+";
              motion_state_.speedup = 1;
              motion_state_.start = 0;
              moving_ = true;
              goback_ = true;     //返回。
            }
            else if(!moving_)
            {
              cmd_vel_.linear.x = 0.0;
              cmd_vel_.linear.y = 0.0;
              cmd_vel_.linear.z = 1.0;          //抬升。
              cmd_vel_.angular.z = 0.0;
            }
          }
/*          else  //异常情况，停止。
          {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.z = 0.0;
          }*/
          break;
        default:
          break;
      }
    }
    else    //需要调整
    {
      switch (odom.pose.tag_num) {
        case 0:
/*          this->alignAlgorithm(odom, error, vx, vy, vw, align_time);
          cmd_vel_.linear.x = vx;
          cmd_vel_.linear.y = vy;
          cmd_vel_.linear.z = 0.0;
          cmd_vel_.angular.z = vw;*/
          //使用轨迹跟踪算法。
       //   trajAlign(odom, error, cmd_vel_, d_const_);
          //使用圆弧轨迹修正算法。
          trajAlign(error, InitErr_, cmd_vel_);
          break;
        case 1:
/*          if(!moving_ && motion_state_.start == 1)
          {
            ROS_WARN("Please align again!");
            alignInit(error,cmd_vel_);
          }
          else if(!moving_ && motion_state_.start == 2)
            alignInit(error, cmd_vel_);
          */
          if(!moving_)
          {
            ROS_WARN("Please align again!");
            alignInit(error, cmd_vel_);
          }

          break;
        case 2:
/*          this->alignAlgorithm(odom, error, vx, vy, vw, align_time);
          cmd_vel_.linear.x = vx;
          cmd_vel_.linear.y = vy;
          cmd_vel_.linear.z = 0.0;
          cmd_vel_.angular.z = vw;*/

          if((motion_state_.expeted_dir == "X+") && (motion_state_.moving_dir != "X+"))
          {
            motion_state_.moving_dir = "X+";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }
          else if(motion_state_.expeted_dir == "Y+" && (motion_state_.moving_dir != "Y+"))
          {
            motion_state_.moving_dir = "Y+";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }
          else if(motion_state_.expeted_dir == "X-" && (motion_state_.moving_dir != "X-"))
          {
            motion_state_.moving_dir = "X-";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }

          else if(motion_state_.expeted_dir == "Y-" && (motion_state_.moving_dir != "Y-"))
          {
            motion_state_.moving_dir = "Y-";
            motion_state_.speedup = (motion_state_.speedup == 1)?-1:1;
          }
          break;
        case 3:
  //        std::cout << "moving_=" << moving_ << std::endl;
          if(!moving_)
          {
          //  stop();
            alignInit(error, cmd_vel_);
          }
          else
            cmd_vel_ = last_cmd_;
          break;
        case 4:
/*          this->alignAlgorithm(odom, error, vx, vy, vw, align_time);
          cmd_vel_.linear.z = 0.0;
          cmd_vel_.angular.z = vw;*/
          if(motion_state_.speedup == 1)    //加速
          {
         //   motion_state_.speedup = -1;
            if(motion_state_.expeted_dir == "X+")
            {
              cmd_vel_.linear.x = TRAVEL_VEL_X;
              cmd_vel_.linear.y = vy;
            }
            else if(motion_state_.expeted_dir == "X-")
            {
              cmd_vel_.linear.x = -TRAVEL_VEL_X;
              cmd_vel_.linear.y = vy;
            }
            else if(motion_state_.expeted_dir == "Y+")
            {
              cmd_vel_.linear.x = vx;
              cmd_vel_.linear.y = TRAVEL_VEL_Y;
            }
            else if(motion_state_.expeted_dir == "Y-")
            {
              cmd_vel_.linear.x = vx;
              cmd_vel_.linear.y = -TRAVEL_VEL_Y;
            }
          }
          else if(motion_state_.speedup == -1)      //减速。
          {
            moving_ = false;
            time_update_ = true;
          //  motion_state_.speedup = 1;
            if(motion_state_.expeted_dir == "X+")
            {
              cmd_vel_.linear.x = START_VEL;
              cmd_vel_.linear.y = vy;
            }
            else if(motion_state_.expeted_dir == "X-")
            {
              cmd_vel_.linear.x = -START_VEL;
              cmd_vel_.linear.y = vy;
            }
            else if(motion_state_.expeted_dir == "Y+")
            {
              cmd_vel_.linear.x = vx;
          //    cmd_vel_.linear.y = START_VEL;
              cmd_vel_.linear.y = 0.1;
          //    std::cout << "speed down" << std::endl;
            }
            else if(motion_state_.expeted_dir == "Y-")
            {
              cmd_vel_.linear.x = vx;
              cmd_vel_.linear.y = -START_VEL;
              motion_state_.start = 2;
            }
          }
          break;
        case 5:
          if(!moving_)
            alignInit(error, cmd_vel_);

          break;
        default:
          break;
      }

    }

   //publish.
    cmd_vel_pub_.publish(cmd_vel_);
    last_cmd_ = cmd_vel_;

    waitMilli(10);
    last_time_ = ros::WallTime::now();
  }
}

void PoseAlign::move()
{
  while(true)
  {
    static short int counter = 0;
    nh_.param("navigation", this->navigation_, false);
    if(!navigation_)
      continue;

    //============= copy data ====================//
    QROdom odom;
    {
      boost::shared_lock<boost::shared_mutex> lock(odom_mutex_);
      odom = qr_odom_;
    }
    Error error;
    bool aligning, x_aligning, y_aligning, yaw_aligning, aligned;
    {
      boost::unique_lock<boost::timed_mutex> err_lock(error_mutex_, boost::try_to_lock);
      if(err_lock.owns_lock() || err_lock.try_lock_for(boost::chrono::milliseconds(10)))
      {
        error = err_;
        err_lock.unlock();
      }
      boost::unique_lock<boost::timed_mutex> state_lock(state_mutex_, boost::try_to_lock);
      if(state_lock.owns_lock() || state_lock.try_lock_for(boost::chrono::milliseconds(10)))
      {
        aligning = aligning_;
        x_aligning = x_aligning_;
        y_aligning = y_aligning_;
        yaw_aligning = yaw_aligning_;
        aligned = aligned_;
        state_lock.unlock();
      }
/*
      printf("aligning=%s\n", aligning==false?"False":"True");
      printf("aligned=%s\n", aligned==false?"False":"True");
      printf("x_aligning=%s\n", x_aligning==false?"False":"True");
      printf("y_aligning=%s\n", y_aligning==false?"False":"True");
      printf("yaw_aligning=%s\n", yaw_aligning==false?"False":"True");
*/
    }
    //moving.
    double goal_x = 0.0;
    double goal_y = 0.0;
    if(!moving_)
    {
      if(odom.pose.tag_num == 1 || odom.pose.tag_num == 3 || odom.pose.tag_num == 5)
      {
        if(odom.pose.tag_num == 5)
          goback_ = true;
        else if(odom.pose.tag_num == 1)
          goback_ = false;

        if(aligning)
          alignInit(error, cmd_vel_);
        else
        {
          cmd_vel_.linear.x = 0.0;
          cmd_vel_.linear.y = 0.0;

          cmd_vel_.angular.x = 0.0;
          cmd_vel_.angular.y = 0.0;
          cmd_vel_.angular.z = 0.0;

          //lift or down.
          if((odom.pose.tag_num == 1) || (odom.pose.tag_num == 5))
          {
            static ros::WallTime lift_time = ros::WallTime::now();    //计时起始处需要设定。
           if(time_update_)
           {
             lift_time = ros::WallTime::now();
             time_update_ = false;
           }
           last_time_ = ros::WallTime::now();
           const unsigned int dt = ((last_time_.sec*1000 + last_time_.nsec/1000000) - (lift_time.sec*1000 + lift_time.nsec/1000000));
            if(dt <= 5000)
            {
              if(motion_state_.lift)
                cmd_vel_.linear.z = -1.0;
              else
                cmd_vel_.linear.z = 1.0;
              ROS_WARN("lift or down.");
            }
            else
            {
              cmd_vel_.linear.z = 0.0;
              motion_state_.lift = motion_state_.lift?false : true;

              //update ref_
              ref_ = odom;  //使用调整结束后的值作为参考值。
              //更新目标点。
              if(goback_ && (counter == goals_num_ -1))
                counter--;
              else
                counter++;
              goal_x = goal_[counter].x;
              goal_y = goal_[counter].y;

              time_update_ = true;
              moving_ = true;       //进入运动状态。
            }
          }
//          if(counter >= goals_num_)
//            counter = 0;
        }
      }
      else
      {
//        std::cout << "Wrong Stop!!!!" << std::endl;
        ROS_ERROR("Wrong Stop!!!!");
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 0.0;
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z = 0.0;
      }
    }
    else        //moving state.
    {
      geometry_msgs::Twist align_vel;
      double temp_x = 0.0;
      double temp_y = 0.0;
      temp_x = goal_x - odom.odom.x;
      temp_y = goal_y - odom.odom.y;
      //到下一个目标点的距离。
      const float Dist2Goal = sqrt(temp_x*temp_x + temp_y*temp_y);
      //agv行驶过的距离。
      const float TravelDist = sqrt((odom.odom.x-ref_.odom.x)*(odom.odom.x-ref_.odom.x) + (odom.odom.y-ref_.odom.y)*(odom.odom.y-ref_.odom.y));


      if((Dist2Goal < position_tolerance_) && ((odom.pose.tag_num == 1) || (odom.pose.tag_num == 3) || (odom.pose.tag_num == 5)))
        moving_ = false;        //进入stop状态。
      else if((TravelDist <= QRDISTANCE ) && ((Dist2Goal < QRDISTANCE) && (odom.pose.tag_num != 0)))   //两段慢速端。
      {
        if(aligning)
        {
          ROS_WARN("Arc aligning.");
          trajAlign(error, InitErr_, align_vel);
        }

        if(fabs(temp_x) > (fabs(temp_y)+1.0))         //moving on X.加1.0意味着fabs(temp_x) >> fabs(temp_y).
        {
          cmd_vel_.linear.x = SIGN(temp_x)*START_VEL;
          cmd_vel_.linear.y = align_vel.linear.y;
        }
        else if((fabs(temp_x)+1.0) < fabs(temp_y))    //moving on Y.
        {
          cmd_vel_.linear.x = align_vel.linear.x;
          cmd_vel_.linear.y = SIGN(temp_x)*START_VEL;
        }
      }
      else      //高速行驶阶段。
      {
        if(aligning)
        {
          ROS_WARN("Arc aligning.");
          trajAlign(error, InitErr_, align_vel);
        }
        if(fabs(temp_x) > fabs(temp_y)) //moving on X.
        {
          cmd_vel_.linear.x = SIGN(temp_x)*TRAVEL_VEL_X;
          cmd_vel_.linear.y = align_vel.linear.y;
        }
        else if(fabs(temp_x) < fabs(temp_y))    //moving on Y.
        {
          cmd_vel_.linear.x = align_vel.linear.x;
          cmd_vel_.linear.y = SIGN(temp_x)*TRAVEL_VEL_Y;
        }
      }
    }
    //publish.
     cmd_vel_pub_.publish(cmd_vel_);

     last_cmd_ = cmd_vel_;
     waitMilli(10);
  }
}
bool PoseAlign::getCmdVel(const Target &goal, const QROdom& cur_pose, geometry_msgs::Twist& vel)
{
  double tempx = 0.0;
  double tempy = 0.0;
  tempx = goal.x - cur_pose.odom.x;
  tempy = goal.y - cur_pose.odom.y;
  geometry_msgs::Twist align_vel;
  //agv行驶过的距离。
  const float travel_dist = sqrt((cur_pose.odom.x-ref_.odom.x)*(cur_pose.odom.x-ref_.odom.x) + (cur_pose.odom.y-ref_.odom.y)*(cur_pose.odom.y-ref_.odom.y));
  //到下一个目标点的距离。
  const float dist2goal = sqrt(tempx*tempx + tempy*tempy);
  short int dir = (fabs(tempx) > fabs(tempy))?SIGN(tempx):SIGN(tempy);  //速度的正负号。
  float vx = 0.0; float vy = 0.0; float vw = 0.0;
  if(fabs(tempx) > fabs(tempy)) //moving on X.
  {
    if((travel_dist <= QRDISTANCE) || (dist2goal <= QRDISTANCE))
    {
      if(cur_pose.pose.tag_num == 3)    //stop.
      {
        if(aligning_)
        {
          alignInit(err_, align_vel);

          vel.linear.x = align_vel.linear.x;
          vel.linear.y = align_vel.linear.y;
          vel.linear.z = align_vel.linear.z;
          vel.angular.x = align_vel.angular.x;
          vel.angular.y = align_vel.angular.y;
          vel.angular.z = align_vel.angular.z;
        }
        else
        {
          ref_ = cur_pose;

        }

      }
      else
      {
        trajAlign(err_, InitErr_, align_vel);
        vel.linear.x = SIGN(tempx)*START_VEL;
        vel.linear.y = align_vel.linear.y;
        vel.angular.z = align_vel.angular.z;
      }

      vel.linear.y = 0.0;
      vel.linear.z = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = 0.0;
      moving_ = true;
    }
//    else if((travel_dist > QRDISTANCE) || (cur_pose.pose.tag_num == 4)) //加速。
    else
    {
      vel.linear.x = SIGN(tempx)*TRAVEL_VEL_X;
      vel.linear.y = 0.0;
      vel.linear.z = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = 0.0;
      moving_ = true;
    }
  }
  else if(fabs(tempx) < fabs(tempy))    //moving on Y.
  {
    if(travel_dist <= QRDISTANCE)
    {
      vel.linear.x = 0.0;
      vel.linear.y = SIGN(tempy)*START_VEL;
      vel.linear.z = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = 0.0;
      moving_ = true;
    }
    else if((travel_dist > QRDISTANCE) || (cur_pose.pose.tag_num == 4)) //加速。
    {
      vel.linear.x = 0.0;
      vel.linear.y = SIGN(tempy)*TRAVEL_VEL_X;
      vel.linear.z = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = 0.0;
      moving_ = true;
    }
  }
  else  //stop
  {
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    moving_ = false;
  }


  return moving_;
}
void PoseAlign::alignAlgorithm(const QROdom &qr_odom, const Error &error, float &vx, float &vy, float &vw, unsigned short int &t)
{
  unsigned short int tx = 0, ty = 0, tYaw = 0;
  if(y_aligning_ && (motion_state_.moving_dir == "X+" || motion_state_.moving_dir == "X-"))   //moving at x direction.
  {
    vx = last_cmd_.linear.x;
    vy = -SIGN(error.y) * ALIGN_VEL;    //y方向上进行微调。

    ty = (unsigned short int)(1000 * fabs(error.y) / ALIGN_VEL);     //调整时间,ms。
  }
  else if(x_aligning_ && (motion_state_.moving_dir == "Y+" || motion_state_.moving_dir == "Y-"))   //moving at y direction.
  {
    vx = -SIGN(error.x) * ALIGN_VEL;
    vy = last_cmd_.linear.y;     //保持当前速度。

    tx = (unsigned short int)(1000 * fabs(error.x) / ALIGN_VEL);     //调整时间。
  }
  else
  {
    vx = last_cmd_.linear.x;
    vy = last_cmd_.linear.y;
  }
  if(yaw_aligning_)
  {
    vw = -SIGN(error.yaw) * ALIGN_VEL;
    tYaw = (unsigned short int)(1000 * fabs(error.yaw) / OMEGA);
  }
  t = std::max(std::max(tx, ty), tYaw);        //调整时间。


/*
  if(!y_aligning_ && !x_aligning_)
  {
    vx = last_cmd_.linear.x;
    vy = last_cmd_.linear.y;
  }
  else if(y_aligning_)
  {

  }
  else if(x_aligning_)
  {


  }*/

}

float PoseAlign::angleMapAndRad(const float &angle)
{
  unsigned short int angle_mapped = 0;  //[-180, 180]
  if(angle >= 180)
    angle_mapped = angle - 360;
  else
    angle_mapped = -angle;
  //degree to radian.
  float angle_rad = (float)angle_mapped * (M_PI/180.0);
  return angle_rad;
}

void PoseAlign::stop()
{
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.linear.z = 0.0;
  cmd_vel_.angular.z = 0.0;
  //publish
  cmd_vel_pub_.publish(cmd_vel_);
  waitMilli(100);
}

void PoseAlign::alignInit(const Error &error, geometry_msgs::Twist &cmd_vel)
{
  float vx = 0.0, vy = 0.0, vw = 0.0;
  if(x_aligning_)
    vx = -SIGN(error.x) * ALIGN_VEL;
  if(y_aligning_)
    vy = -SIGN(error.y) * ALIGN_VEL;
  if(yaw_aligning_)
    vw = -SIGN(error.yaw) * ALIGN_VEL;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.z = vw;
}
//轨迹修正算法实现。
void PoseAlign::trajAlign(const QROdom &qr_odom, const Error &error, geometry_msgs::Twist &vel, float dist)
{
/*  float v=0.0;
  if(motion_state_.expeted_dir == "Y+")
  {
    if(motion_state_.expeted_dir != motion_state_.moving_dir)
      v = START_VEL;
    else if(motion_state_.expeted_dir == motion_state_.moving_dir)
    {
      if(motion_state_.speedup == 1)
        v = TRAVEL_VEL_Y;
      else
        v = START_VEL;
    }
  }
  else if(motion_state_.expeted_dir == "X+")
  {
    if(motion_state_.expeted_dir != motion_state_.moving_dir)
        v = START_VEL;
    else if(motion_state_.expeted_dir == motion_state_.moving_dir)
    {
      if(motion_state_.speedup == 1)
        v = TRAVEL_VEL_X;
      else
        v = START_VEL;
    }
  }
  else if(motion_state_.expeted_dir == "Y-")
  {
    if(motion_state_.expeted_dir != motion_state_.moving_dir)
        v = -START_VEL;
    else if(motion_state_.expeted_dir == motion_state_.moving_dir)
    {
      if(motion_state_.speedup == 1)
        v = -TRAVEL_VEL_Y;
      else
        v = -START_VEL;
    }
  }
  else if(motion_state_.expeted_dir == "X-")
  {
    if(motion_state_.expeted_dir != motion_state_.moving_dir)
        v = -START_VEL;
    else if(motion_state_.expeted_dir == motion_state_.moving_dir)
    {
      if(motion_state_.speedup == 1)
        v = -TRAVEL_VEL_X;
      else
        v = -START_VEL;
    }
  }*/

  if(y_aligning_ && (motion_state_.expeted_dir == "X+" || motion_state_.expeted_dir == "X-"))   //moving at x direction.
  {
    float d = fabs(dist - qr_odom.odom.x);
    float er = error.y;                         //必须保证误差数据的正确性。
    float temp = (2*d*er)/(er*er+d*d);

//    vel.linear.y = -1*fabs(v)*tan(asin(temp));
//    vel.linear.x = v;
    float align_vel = -1*fabs(last_cmd_.linear.x)*tan(asin(temp));
    vel.linear.y = (fabs(align_vel)>0.1)?SIGN(align_vel)*0.1:align_vel;
    vel.linear.x = last_cmd_.linear.x;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
  }
  else if(x_aligning_ && (motion_state_.expeted_dir == "Y+" || motion_state_.expeted_dir == "Y-"))   //moving at y direction.
  {
    float d = fabs(dist - qr_odom.odom.y);
    float er = error.x;
 //   printf("error = %.3f\n",er);
    float temp = (2*d*er)/(er*er+d*d);
//    vel.linear.x = -1*fabs(v)*tan(asin(temp));
//    vel.linear.y = v;
    float align_vel = -1*fabs(last_cmd_.linear.y)*tan(asin(temp));
    vel.linear.x = (fabs(align_vel)>0.1)?SIGN(align_vel)*0.1:align_vel;
    vel.linear.y = last_cmd_.linear.y;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
  }
  else
  {
    vel = last_cmd_;
    printf("last_cmd!\n");
  }

  if(yaw_aligning_)
    vel.angular.z = SIGN(error.yaw) * 0.05;
  else
    vel.angular.z = 0.0;

}

void PoseAlign::trajAlign(const Error &error, const Error &init_error, geometry_msgs::Twist &vel)
{
  const float VelAlign = 0.02;//最大调整速度。
  if((motion_state_.expeted_dir == "X+") || (motion_state_.expeted_dir == "X-"))
  {
    const float VelTravel = last_cmd_.linear.x;
    float vy = 0.0;
    if(y_aligning_)
    {
      if(!traj_aligning_)
      {
        InitErr_.x = error.x;
        InitErr_.y = error.y;
        InitErr_.yaw = error.yaw;
 //     printf("error======(%.3f, %.3f, %.3f)\n",error.x, error.y, error.yaw);
        traj_aligning_ = true;
      }
      printf("error======(%.3f, %.3f, %.3f)\n",error.x, error.y, error.yaw);
      const float InitTheta = fabs(atan(VelAlign/VelTravel));     //rad.
      const float k = (1-cos(InitTheta))/fabs(init_error.y);
      float a = 1-k*fabs(error.y);
      a = fabs(a)>1?SIGN(a):a;
      float theta = fabs(acos(a));
      theta = (theta > InitTheta)?InitTheta:theta;
      vy = -1*SIGN(error.y)*fabs(VelTravel)*tan(theta);

/*
      printf("InitTheta =%.3f\n", InitTheta);
      printf("error_y =$$$$$$$$$$$%.3f\n", error.y);
      printf("init_error_y =%.3f\n", init_error.y);
      printf("k =%.3f\n", k);
      printf("theta =%.3f\n", theta);
      printf("vy =%.3f\n", vy);
*/
    }
    else
      vy = 0.0;
    //cmd_vel
    vel.linear.x = VelTravel;
    vel.linear.y = vy;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
  }
  else if((motion_state_.expeted_dir == "Y+") || (motion_state_.expeted_dir == "Y-"))
  {
    const float VelTravel = last_cmd_.linear.y;
    float vx = 0.0;
    if(x_aligning_)
    {
      if(!traj_aligning_)
      {
        InitErr_.x = error.x;
        InitErr_.y = error.y;
        InitErr_.yaw = error.yaw;
 //       printf("error=(%.3f, %.3f, %.3f)\n",error.x, error.y, error.yaw);
        traj_aligning_ = true;
      }
      const float VelTravel = last_cmd_.linear.y;
      const float InitTheta = fabs(atan(VelAlign/VelTravel));     //rad.
      const float k = (1-cos(InitTheta))/fabs(init_error.x);
      float a = 1-k*fabs(error.x);
      a = fabs(a)>1?SIGN(a):a;
      float theta = fabs(acos(a));
      theta = (theta > InitTheta)?InitTheta:theta;
      float vx = -1*SIGN(error.x)*fabs(VelTravel)*tan(theta);
/*
      printf("InitTheta =%.3f\n", InitTheta);
      printf("error_x =%.3f\n", error.x);
      printf("init_error_x =%.3f\n", init_error.x);
      printf("k =%.3f\n", k);
      printf("theta =%.3f\n", theta);
      printf("vx =%.3f\n", vx);
 */
    }
    else
      vx = 0.0;
    //cmd_vel
    vel.linear.x = vx;
    vel.linear.y = VelTravel;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
  }

/*
  if(y_aligning_ && (motion_state_.expeted_dir == "X+" || motion_state_.expeted_dir == "X-"))   //moving at x direction.
  {
    if(!traj_aligning_)
    {
      InitErr_.x = error.x;
      InitErr_.y = error.y;
      InitErr_.yaw = error.yaw;
      printf("error=(%.3f, %.3f, %.3f)\n",error.x, error.y, error.yaw);
      traj_aligning_ = true;
    }
    const float VelTravel = last_cmd_.linear.x;
    const float InitTheta = fabs(atan(VelAlign/VelTravel));     //rad.
    const float k = (1-cos(InitTheta))/fabs(init_error.y);
    float a = 1-k*fabs(error.y);
    a = fabs(a)>1?SIGN(a):a;
    float theta = fabs(acos(a));
    theta = (theta > InitTheta)?InitTheta:theta;
    float vy = -1*SIGN(error.y)*fabs(VelTravel)*tan(theta);

    printf("InitTheta =%.3f\n", InitTheta);
    printf("error_y ========%.3f\n", error.y);
    printf("init_error_y =%.3f\n", init_error.y);
    printf("k =%.3f\n", k);
    printf("theta =%.3f\n", theta);
    printf("vy =%.3f\n", vy);
    //cmd_vel
    vel.linear.x = VelTravel;
    vel.linear.y = vy;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
  }
  else if(x_aligning_ && (motion_state_.expeted_dir == "Y+" || motion_state_.expeted_dir == "Y-"))   //moving at y direction.
  {
    if(!traj_aligning_)
    {
      InitErr_.x = error.x;
      InitErr_.y = error.y;
      InitErr_.yaw = error.yaw;
      printf("error=(%.3f, %.3f, %.3f)\n",error.x, error.y, error.yaw);
      traj_aligning_ = true;
    }
    const float VelTravel = last_cmd_.linear.y;
    const float InitTheta = fabs(atan(VelAlign/VelTravel));     //rad.
    const float k = (1-cos(InitTheta))/fabs(init_error.x);
    float a = 1-k*fabs(error.x);
    a = fabs(a)>1?SIGN(a):a;
    float theta = fabs(acos(a));
    theta = (theta > InitTheta)?InitTheta:theta;
    float vx = -1*SIGN(error.x)*fabs(VelTravel)*tan(theta);

    printf("InitTheta =%.3f\n", InitTheta);
    printf("error_x =%.3f\n", error.x);
    printf("init_error_x =%.3f\n", init_error.x);
    printf("k =%.3f\n", k);
    printf("theta =%.3f\n", theta);
    printf("vx =%.3f\n", vx);
    //cmd_vel
    vel.linear.x = vx;
    vel.linear.y = VelTravel;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;

  }
 */
  else
  {
    vel = last_cmd_;
  }
  if(yaw_aligning_)
    vel.angular.z = -SIGN(error.yaw) * 0.05;
  else
    vel.angular.z = 0.0;
}


