#pragma once
#include <iostream>
class IState
{
public:
	virtual std::string getCurState() const = 0;
};