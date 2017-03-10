/*
 * events.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_EVENTS_H_
#define INCLUDE_AGV_STATEMACHINE_EVENTS_H_

#include <boost/statechart/event.hpp>

class EvAlign : public sc::event<EvAlign>
{
public:
  EvAlign();
  ~EvAlign();
};

class EvTest : public sc::event<EvTest>
{
public:
  EvTest();
  ~EvTest();
};

class EvLiftDown : public sc::event<EvLiftDown>
{
public:
  EvLiftDown();
  ~EvLiftDown();
};

class EvPath : public sc::event<EvPath>
{
public:
  EvPath();
  ~EvPath();
};

#endif /* INCLUDE_AGV_STATEMACHINE_EVENTS_H_ */
