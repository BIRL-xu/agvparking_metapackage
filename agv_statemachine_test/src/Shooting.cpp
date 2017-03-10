#include "agv_statemachine/Shooting.h"
#include "agv_statemachine/orthogonal_states.h"
/*
result Focusing::react(const EvInFocus& evt)
{
	return transit<Focused>(&Shooting::DisplayFocused, evt);	//�Զ�����Ӧ��Ӧ����Ӧ�������÷�����
}

result Focused::react(const EvShutterFull&)
{
	if (context<Camera>().IsMemoryAvailable())
	{
		return transit<Storing>();
	}
	else
	{
		std::cout << "Camera memory full. Please wait...\n";
	//	return transit<Focused>();
		return discard_event();	//�������¼���
	}
}
*/

//=============== Shooting ===================//
Shooting::Shooting()
{
	std::cout << "Enter Shooting!\n";
}
Shooting::~Shooting()
{
	std::cout << "Exit Shooting!\n";
}

//=============== Focusing ===================//
Focusing::Focusing()
{
	std::cout << "Enter Focusing!\n";
}
Focusing::~Focusing()
{
	std::cout << "Exit Focusing!\n";
}

std::string Focusing::getCurState() const
{
	return std::string("Focusing");
}

result Focusing::react(const EvInFocus& evInFocus)
{
	std::cout << "Focusing::react(const EvInFocus& evInFocus)\n";
	return transit<Focused>(&Camera::displayFocused, evInFocus);
}
//=============== Focused ===================//
Focused::Focused()
{
	std::cout << "Enter Focused!\n";
}
Focused::~Focused()
{
	std::cout << "Exit Focused!\n";
}

std::string Focused::getCurState() const
{
	return std::string("Focused");
}
result Focused::react(const EvShutterFull& evShutterFull)
{
	std::cout << "Focused::react(const EvShutterFull & evShutterFull )\n";  

	if (context<Camera>().IsMemoryAvailable())
	{
		std::cout << "Guard: isMemAvail() is true.\n";
		return transit<Storing>(&Camera::allocMem, evShutterFull);
	}
	else
	{
		std::cout << "Guard: isMemAvail() is false.\n";
		return discard_event();
	}
}

//=============== Storing ===================//
Storing::Storing()
{
	std::cout << "Enter Storing!\n";
}
Storing::~Storing()
{
	std::cout << "Exit Storing!\n";
}

std::string Storing::getCurState() const
{
	return std::string("Storing");
}
result Storing::react(const EvStored& evStored)
{
	std::cout << "Storing::react(const EvStored & evStored).\n";

//	context<Camera>().process_event(EvShutterRelease());
//	return transit<NotShooting>();

	return forward_event();
}

result Storing::react(const EvPlayer& evPlayer)
{
  std::cout << "Storing::react(const EvPlayer& evPlayer).\n";

  return transit<OrthogonalStates>();
}

