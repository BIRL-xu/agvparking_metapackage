#ifndef INCLUDE_NOT_SHOOTING_H_H_
#define INCLUDE_NOT_SHOOTING_H_H_
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/deep_history.hpp>
//#include <boost/statechart/shallow_history.hpp>

#include "agv_statemachine/istate.h"
#include "agv_statemachine/events.h"
#include "agv_statemachine/camera.h"

class Idle;
class NotShooting : public simple_state<NotShooting, Camera, Idle, has_deep_history>      //添加深历史信息。
{
public:
	//�ö���ʽֻ��˵���¼�EvShutterHalf������ĳ���£����Ǿ�����ʲô����Ҫʹ��react��Ա��������ʵ�֡�
	typedef custom_reaction<EvShutterHalf> reactions;
	NotShooting();
	~NotShooting();


	result react(const EvShutterHalf& evShtterHalf);	//reactģ�庯���Ķ�̬��

};

class Idle :public simple_state<Idle, NotShooting>, public IState
{
public:
	typedef custom_reaction<EvConfig> reactions;
	Idle();
	~Idle();

	virtual std::string getCurState() const;
	result react(const EvConfig&);

};

class Configuring :public simple_state<Configuring, NotShooting>, public IState
{
public:
  typedef boost::statechart::transition<EvConfig, Idle, Camera, &Camera::powerSavingMode> reactions;
  Configuring();
  ~Configuring();

  virtual std::string getCurState() const;
};
#endif
