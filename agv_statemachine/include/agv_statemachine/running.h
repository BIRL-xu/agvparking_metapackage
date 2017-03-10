/*
 * running.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_RUNNING_H_
#define INCLUDE_AGV_STATEMACHINE_RUNNING_H_

#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/mpl/list.hpp>

#include "agv_statemachine/states_storage.h"
#include "agv_statemachine/events.h"
#include "agv_statemachine/agv_statemachine.h"

class AlignArcTraj;
class LowVel;

class Running : public sc::simple_state<Running, agvStateMachine, boost::mpl::list<LowVel, AlignArcTraj>, sc::has_deep_history>
{
public:
  Running();
  ~Running();

};
/*class Running : public sc::simple_state<Running, agvStateMachine, boost::mpl::list<YPlus, LowVel>, sc::has_deep_history>
{
public:
  Running();
  ~Running();

};

class YPlus : public sc::simple_state<YPlus, Running::orthogonal<0> >, public StatesStorage
{
public:
  YPlus();
  ~YPlus();

};
class YMinus : public sc::simple_state<YMinus, Running::orthogonal<0> >, public StatesStorage
{
public:
  YMinus();
  ~YMinus();

};

class XPlus : public sc::simple_state<XPlus, Running::orthogonal<0> >, public StatesStorage
{
public:
  XPlus();
  ~XPlus();

};

class XMinus : public sc::simple_state<XMinus, Running::orthogonal<0> >, public StatesStorage
{
public:
  XMinus();
  ~XMinus();
};*/

class LowVel : public sc::simple_state<LowVel, Running::orthogonal<0> >, public StatesStorage
{
public:
  LowVel();
  ~LowVel();
};

class HighVel : public sc::simple_state<HighVel, Running::orthogonal<0> >, public StatesStorage
{
public:
  HighVel();
  ~HighVel();
};

class AlignArcTraj : public sc::simple_state<AlignArcTraj, Running::orthogonal<1> >
{
public:
  AlignArcTraj();
  ~AlignArcTraj();
};
#endif /* INCLUDE_AGV_STATEMACHINE_RUNNING_H_ */
