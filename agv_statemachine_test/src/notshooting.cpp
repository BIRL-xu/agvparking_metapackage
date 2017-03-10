#include "agv_statemachine/notshooting.h"
#include "agv_statemachine/Shooting.h"

//================= NotShooting =====================//
NotShooting::NotShooting()
{
	std::cout << "Enter  NotShooting!\n";
}
NotShooting::~NotShooting()
{
	std::cout << "Exit  NotShooting!\n";
}

result NotShooting::react(const EvShutterHalf & evShutterHalf)
{
	std::cout << "NotShooting::react(const EvShutterHalf & evShutterHalf).\n";
	if (context< Camera >().IsBatteryLow())
	{
		std::cout << "Guard: IsBatteryLow() is true.\n";
		//Yuanguo: We cannot react to the event ourselves, so we forward it  
		//to our outer state (this is also the default if a state  
		//defines no reaction for a given event).  
		return forward_event();
	}
	else
	{
		std::cout << "Guard: isBatteryLow() is false.\n";
		return transit<Shooting>();  //no transition action  
	}
}

//================= Idle =====================//
Idle::Idle()
{
	std::cout << "Enter Idle!\n";
}
Idle::~Idle()
{
	std::cout << "Exit Idle!\n";
}

std::string Idle::getCurState() const
{
	return std::string("Idle");
}

result Idle::react(const EvConfig& evConfig)
{
	std::cout << "Idle::react(const EvConfig& evConfig).\n";
	
	return transit<Configuring>();
}

//================= Configuring =====================//
Configuring::Configuring()
{
	std::cout << "Enter Configuring!\n";
}
Configuring::~Configuring()
{
	std::cout << "Exit Configuring!\n";
}

std::string Configuring::getCurState() const
{
	return std::string("Configuring");
}
