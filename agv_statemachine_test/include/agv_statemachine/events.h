#ifndef INCLUDE_EVENTS_H_H_
#define INCLUDE_EVENTS_H_H_
#include <boost/statechart/event.hpp>
using namespace boost::statechart;

//Events declaretion.

class EvConfig : public event<EvConfig> 
{
public:
	EvConfig();
	~EvConfig();
};

class EvShutterHalf : public event<EvShutterHalf> 
{
public:
	EvShutterHalf();
	~EvShutterHalf();
};
class EvShutterFull : public event<EvShutterFull> 
{
public:
	EvShutterFull();
	EvShutterFull(const EvShutterFull& other);
	~EvShutterFull();

};
class EvShutterRelease : public event<EvShutterRelease> 
{
public:
	EvShutterRelease();
	~EvShutterRelease();
};

class EvInFocus : public event< EvInFocus >
{
public:
	EvInFocus();
	~EvInFocus();
};

class EvStored : public event< EvStored >
{
public:
	EvStored();
	~EvStored();
};

class EvPlayer: public event< EvPlayer >
{
public:
  EvPlayer();
  ~EvPlayer();
};

class EvPlayerRelease: public event <EvPlayerRelease>
{
public:
  EvPlayerRelease();
  ~EvPlayerRelease();
};

#endif
