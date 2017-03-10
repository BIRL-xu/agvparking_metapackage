#ifndef INCLUDE_SHOOTING_H_H_
#define INCLUDE_SHOOTING_H_H_
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/deferral.hpp>
#include <boost/mpl/list.hpp>

#include "agv_statemachine/istate.h"
#include "agv_statemachine/events.h"
#include "agv_statemachine/camera.h"
#include "agv_statemachine/notshooting.h"
class Focusing;

class Shooting : public simple_state<Shooting, Camera, Focusing>
{
public:
  typedef transition< EvShutterRelease, boost::statechart::deep_history<Idle> > reactions;      //进入历史状态。
  Shooting();
  ~Shooting();
};

class Focusing : public simple_state<Focusing, Shooting>, public IState
{
public:

	typedef boost::mpl::list<
		custom_reaction<EvInFocus>,
		deferral<EvShutterFull>	                //延迟事件。
	> reactions;

	//tansition�ĵ��÷�ʽ��
	//typedef sc::transition<EvInFocus, Focused,
	//	Shooting, &Shooting::DisplayFocused> reactions;


	Focusing();
	virtual~Focusing();

	virtual std::string getCurState() const;

	result react(const EvInFocus &);
};

class Focused :public simple_state<Focused, Shooting>, public IState
{
public:
	typedef custom_reaction<EvShutterFull> reactions;

	Focused();
	virtual ~Focused();

	virtual std::string getCurState() const;
	result react(const EvShutterFull &);
};

class Storing :public simple_state<Storing, Shooting>, public IState
{
public:
        typedef boost::mpl::list<
            custom_reaction<EvStored>,
            custom_reaction<EvPlayer> > reactions;    //自定义事件不能添加历史状态。

	Storing();
	virtual ~Storing();
	virtual std::string getCurState() const;

	result react(const EvStored& evStored);
	result react(const EvPlayer& evPlayer);
};
#endif
