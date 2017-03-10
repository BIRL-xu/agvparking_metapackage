/*
 * compositestop.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */


#include "agv_statemachine/compositestop.h"
#include "agv_statemachine/states_macro.h"
using namespace std;

CompositeStop::CompositeStop():error_(true)
{
  cout << "Construct CompositeStop." << endl;

}

CompositeStop::~CompositeStop()
{
  cout << "Destruct CompositeStop." << endl;

}

Stop::Stop()
{
  cout << "Enter Stop." << endl;
}
Stop::~Stop()
{
  cout << "Exit Stop." << endl;
}
sc::result Stop::react(const EvAlign& evAlign)
{
  EvTest evTest;

  zeroVel();
  if(!context<CompositeStop>().error_)
    return transit<LiftOrDown>();
  else
    return transit<AlignAtPlace>(&AlignAtPlace::alignAtPlace, evTest);
}
unsigned int Stop::getCurState() const
{
  return STOP;
}

AlignAtPlace::AlignAtPlace()
{
  cout << "Enter AlignAtPlace." << endl;
}
AlignAtPlace::~AlignAtPlace()
{
  cout << "Exit AlignAtPlace." << endl;
}
void AlignAtPlace::alignAtPlace(const EvTest& evTest)
{
  while(context<CompositeStop>().error_)
  {
    cout << "alignning at place." << endl;

  }
  //set odom reference.
}

OdomRefUpdate::OdomRefUpdate()
{
  cout << "Enter OdomRefUpdate." << endl;
}
OdomRefUpdate::~OdomRefUpdate()
{
  cout << "Exit OdomRefUpdate." << endl;
}


LiftOrDown::LiftOrDown()
{
  cout << "Construct LiftOrDown." << endl;
}
LiftOrDown::~LiftOrDown()
{
  cout << "Destruct LiftOrDown." << endl;
}
sc::result LiftOrDown::react(const EvPath& evPath)
{
  if(isNewGoal())
  {
    return transit<GetPath>();
  }
  else
  {
    return transit<Running>();
  }
}

Lift::Lift()
{
  cout << "Enter Lift." << endl;
  timer_ =  nh_.createTimer(ros::Duration(1), boost::bind(&Lift::lifting, this));
}
Lift::~Lift()
{
  cout << "Exit Lift." << endl;
}
void Lift::lifting()
{
  static int counter = 0;counter++;
  //TODO lifting.
  if(counter == 5)
  {
    timer_.stop();
    context<agvStateMachine>().process_event(EvLiftDown());
  }
  //ros 异步spinner.
}

Down::Down()
{
  cout << "Enter Down." << endl;
  timer_ =  nh_.createTimer(ros::Duration(1), boost::bind(&Down::downing, this));
}
Down::~Down()
{
  cout << "Exit Down." << endl;
}
void Down::downing()
{
  static int counter = 0;counter++;
  //TODO Downing.
  if(counter == 5)
  {
    timer_.stop();
    context<agvStateMachine>().process_event(EvLiftDown());
  }
  //ros 异步spinner.
}

GetPath::GetPath()
{
  cout << "Enter GetPath." << endl;
}
GetPath::~GetPath()
{
  cout << "Exit GetPath." << endl;
}








