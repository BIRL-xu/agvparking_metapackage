/*
 * agv_statemachine.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_AGV_STATEMACHINE_H_
#define INCLUDE_AGV_STATEMACHINE_AGV_STATEMACHINE_H_

#include <boost/statechart/state_machine.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>

namespace sc = boost::statechart;
class CompositeStop;

class agvStateMachine : public sc::state_machine<agvStateMachine, CompositeStop>
{
public:
  agvStateMachine();
  ~agvStateMachine();

private:
  void run();

  boost::shared_ptr<boost::thread>  SM_thread_;

};





#endif /* INCLUDE_AGV_STATEMACHINE_AGV_STATEMACHINE_H_ */
