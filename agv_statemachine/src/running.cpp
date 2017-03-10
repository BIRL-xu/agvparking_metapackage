/*
 * running.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: paul
 */
#include "agv_statemachine/running.h"
#include "agv_statemachine/states_storage.h"
using namespace std;
Running::Running()
{
  cout << "Construct Running." << endl;

}
Running::~Running()
{
  cout << "Destruct Running." << endl;
}

LowVel::LowVel()
{
  cout << "Enter LowVel." << endl;
}
LowVel::~LowVel()
{
  cout << "Exit LowVel." << endl;
}

HighVel::HighVel()
{
  cout << "Enter HighVel." << endl;

}
HighVel::~HighVel()
{
  cout << "Exit HighVel." << endl;

}

AlignArcTraj::AlignArcTraj()
{
  cout << "Enter AlignArcTraj." << endl;
}
AlignArcTraj::~AlignArcTraj()
{
  cout << "Exit AlignArcTraj." << endl;
}
