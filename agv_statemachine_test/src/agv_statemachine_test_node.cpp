/*
 * agv_statemachine_node.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: paul
 */
#include "ros/ros.h"
#include "agv_statemachine/camera.h"
#include "agv_statemachine/notshooting.h"
#include "agv_statemachine/Shooting.h"
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

using namespace std;
void startThread()
{
  while(true)
  {
    boost::shared_ptr<Camera> ptr(new Camera());

    ptr->initiate();
    ptr->process_event(EvConfig());     //模拟按config键。
    cout<< ptr->getCurState() << endl;
  }

}
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "agv_statemachine_node");
//使用线程跑状态机。
  boost::shared_ptr<boost::thread> thrd_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&startThread)));
  /*
  Camera camera;
  camera.initiate();

  cout<< camera.getCurState() << endl;

  camera.process_event(EvConfig());     //模拟按config键。
  cout<< camera.getCurState() << endl;

//  camera.process_event(EvConfig());
//  cout<< camera.getCurState() << endl;

  camera.process_event(EvShutterHalf());        //模拟半按快门。
  cout<< camera.getCurState() << endl;

  cout<<"Press Shutter Full before focused"<<endl;
  camera.process_event(EvShutterFull());  //在对焦完成之前，模拟全按快门
  cout<<camera.getCurState()<<endl;

  camera.process_event(EvInFocus());      //模拟对焦完成事件
  cout<<camera.getCurState()<<endl;

//  camera.process_event(EvShutterRelease()); //模拟释放快门
//  cout<<camera.getCurState()<<endl;

  camera.process_event(EvStored());         //模拟存储完成
  cout<<camera.getCurState()<<endl;

  camera.process_event(EvPlayer());     //进入正交状态。
  cout<<camera.getCurState()<<endl;
  cout<<camera.getCurState()<<endl;

  camera.process_event(EvPlayerRelease());     //退出正交状态。
  cout<<camera.getCurState()<<endl;*/
  ros::spin();
  return 0;
}


