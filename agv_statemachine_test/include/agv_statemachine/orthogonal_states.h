/*
 * orthogonal_states.h
 *
 *  Created on: Mar 1, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_ORTHOGONAL_STATES_H_
#define INCLUDE_AGV_STATEMACHINE_ORTHOGONAL_STATES_H_

#include <boost/statechart/simple_state.hpp>
#include <boost/mpl/list.hpp>

#include "agv_statemachine/camera.h"
#include "agv_statemachine/events.h"
#include "agv_statemachine/notshooting.h"

class Video;
class Music;
class OrthogonalStates:public simple_state<OrthogonalStates, Camera,
  boost::mpl::list<Video, Music> >
{
public:
  typedef transition<EvPlayerRelease, NotShooting> reactions;

  OrthogonalStates();
  ~OrthogonalStates();

};


class Video:public simple_state<Video, OrthogonalStates::orthogonal<0> >, public IState
{
public:
  Video();
  ~Video();

  virtual std::string getCurState() const;
};

class Music:public simple_state<Music, OrthogonalStates::orthogonal<1> >, public IState
{
public:
  Music();
  ~Music();

  virtual std::string getCurState() const;
};
#endif /* INCLUDE_AGV_STATEMACHINE_ORTHOGONAL_STATES_H_ */
