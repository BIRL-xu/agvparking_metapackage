/*
 * compositestop.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_COMPOSITESTOP_H_
#define INCLUDE_AGV_STATEMACHINE_COMPOSITESTOP_H_

#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/mpl/list.hpp>

#include "agv_statemachine/states_storage.h"
#include "agv_statemachine/events.h"
#include "agv_statemachine/agv_statemachine.h"

class Stop;

class CompositeStop : public sc::simple_state<CompositeStop, agvStateMachine, Stop>
{
public:
  CompositeStop();
  ~CompositeStop();


private:

  bool error_;
};

class Stop : public sc::simple_state<Stop, CompositeStop>, public StatesStorage
{
public:
  Stop();
  virtual ~Stop();

  typedef sc::custom_reaction<EvAlign> reactions;

  sc::result react(const EvAlign& evAlign);

  virtual unsigned int getCurState() const;

private:
  void zeroVel();
};

class AlignAtPlace : public sc::simple_state<AlignAtPlace, CompositeStop>, public StatesStorage
{
public:
  AlignAtPlace();
  ~AlignAtPlace();
  void alignAtPlace(const EvTest& evTest);
};

class OdomRefUpdate : public sc::simple_state<OdomRefUpdate, CompositeStop>, public StatesStorage
{
public:
  OdomRefUpdate();
  ~OdomRefUpdate();
};

class Lift;

class LiftOrDown : public sc::simple_state<LiftOrDown, CompositeStop, Lift, sc::has_deep_history>
{
public:
  LiftOrDown();
  ~LiftOrDown();

  typedef sc::custom_reaction<EvPath> reactions;

  bool isNewGoal(){return true;}

  sc::result react(const EvPath& evPath);

};

class Lift : public sc::simple_state<Lift, LiftOrDown>, public StatesStorage
{
public:
  Lift();
  ~Lift();

  typedef sc::transition<EvLiftDown, Down> reactions;

  ros::NodeHandle nh_;
  ros::Timer timer_;
private:
  void lifting();
};

class Down : public sc::simple_state<Down, LiftOrDown>, public StatesStorage
{
public:
  Down();
  ~Down();

  typedef sc::transition<EvLiftDown, Lift> reactions;

  ros::NodeHandle nh_;
  ros::Timer timer_;

private:
  void downing();
};

class GetPath : public sc::simple_state<GetPath, CompositeStop>, public StatesStorage
{
public:
  GetPath();
  ~GetPath();
};
#endif /* INCLUDE_AGV_STATEMACHINE_COMPOSITESTOP_H_ */
