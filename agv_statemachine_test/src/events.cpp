#include <iostream>

#include "agv_statemachine/events.h"
//============= EvConfig===============//
EvConfig::EvConfig()
{
	std::cout << "Construct EvConfig.\n";
}
EvConfig::~EvConfig()
{
	std::cout << "Destruct EvConfig.\n";
}
//============= EvShutterHalf===============//
EvShutterHalf::EvShutterHalf()
{
	std::cout << "Construct EvShutterHalf.\n";
}
EvShutterHalf::~EvShutterHalf()
{
	std::cout << "Dstruct EvShutterHalf.\n";
}
//============= EvShutterFull===============//
EvShutterFull::EvShutterFull()
{
	std::cout << "Construct EvShutterFull.\n";
}
EvShutterFull::EvShutterFull(const EvShutterFull& other)
{
  std::cout << "Copy Construct EvShutterFull.\n";
}
EvShutterFull::~EvShutterFull()
{
	std::cout << "Dstruct EvShutterFull.\n";
}
//============= EvShutterRelease===============//
EvShutterRelease::EvShutterRelease()
{
	std::cout << "Construct EvShutterRelease.\n";
}
EvShutterRelease::~EvShutterRelease()
{
	std::cout << "Dstruct EvShutterRelease.\n";
}
//============= EvInFocus==============//
EvInFocus::EvInFocus()
{
	std::cout << "Construct EvInFocus.\n";
}
EvInFocus::~EvInFocus()
{
	std::cout << "Dstruct EvInFocus.\n";
}
//============= EvStored==============//
EvStored::EvStored()
{
	std::cout << "Construct EvStored.\n";
}
EvStored::~EvStored()
{
	std::cout << "Dstruct EvStored.\n";
}

EvPlayer::EvPlayer()
{
  std::cout << "Construct EvPlayer.\n";
}
EvPlayer::~EvPlayer()
{
  std::cout << "Destruct EvPlayer.\n";
}

EvPlayerRelease::EvPlayerRelease()
{
  std::cout << "Construct EvPlayerRelease.\n";
}
EvPlayerRelease::~EvPlayerRelease()
{
  std::cout << "Destruct EvPlayerRelease.\n";
}
