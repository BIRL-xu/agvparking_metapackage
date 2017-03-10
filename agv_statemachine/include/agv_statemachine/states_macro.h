/*
 * states_macro.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_STATES_MACRO_H_
#define INCLUDE_AGV_STATEMACHINE_STATES_MACRO_H_


//======== states MACRO ==============//
//1.compositestop
#define COMPOSITESTOP   1
#define STOP            2
#define ALIGNATPLACE    3
#define ODOMREFUPDATE   4
#define LIFTORDOWN      5
#define LIFT            6
#define DOWN            7
#define GETPATH         8


//2.Running
#define RUNNING         9
#define Y+              10
#define Y-              11
#define X+              12
#define X-              13
#define LOWVEL          14
#define HIGHVEL         15
#define ALIGNARCTRAJ    16



#endif /* INCLUDE_AGV_STATEMACHINE_STATES_MACRO_H_ */
