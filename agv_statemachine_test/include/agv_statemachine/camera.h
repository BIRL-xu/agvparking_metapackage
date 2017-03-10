#ifndef INCLUDE_CAMERA_H_H_
#define INCLUDE_CAMERA_H_H_
#include <boost/statechart/state_machine.hpp>
#include "agv_statemachine/events.h"

//Initial State Declaretion.
class NotShooting;

class Camera :public state_machine<Camera, NotShooting>
{
public:
	Camera();
	~Camera();

	std::string getCurState() const;
	bool IsMemoryAvailable() const { return true; }
	bool IsBatteryLow() const { return false; }

	//transition actions
	void displayFocused(const EvInFocus& evInFocus);
	void allocMem(const EvShutterFull& evShutterFull);
	void powerSavingMode(const EvConfig& evConfig);
};
#endif
