/*
 * agv_statemachine.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#include "agv_statemachine/agv_statemachine.h"
using namespace std;
agvStateMachine::agvStateMachine()
{
  cout << "Construct agvStateMachine." << endl;
  //subscribe ros topics.

  SM_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&agvStateMachine::run, this)));

}
agvStateMachine::~agvStateMachine()
{
  cout << "Destruct agvStateMachine." << endl;
}

void agvStateMachine::run()
{
  while(!context<agvStateMachine>().terminated())
  {
    //回调函数。

  }

}
