
#include "agv_statemachine/camera.h"
#include "agv_statemachine/notshooting.h"
#include "agv_statemachine/Shooting.h"

Camera::Camera()
{
	std::cout << "Construct Camera!\n";
}
Camera::~Camera()
{
	std::cout << "Destruct Camera!\n";
}

std::string Camera::getCurState() const
{
	return std::string("CurrentState ------> ") + state_cast<const IState&>().getCurState();
}

void Camera::displayFocused(const EvInFocus & evInFocus)
{
	std::cout << "[Transition Action]: Camera focused on objects.\n";
}
void Camera::allocMem(const EvShutterFull & evShutterFull)
{
	std::cout << "[Transition Action]: Memory allocated for storing the picture.\n";
}
void Camera::powerSavingMode(const EvConfig & evConfig)
{
	std::cout << "[Transition Action]: Camera goes into Power Saving Mode.\n";
}

