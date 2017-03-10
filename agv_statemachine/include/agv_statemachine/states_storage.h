/*
 * states_storage.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_STATES_STORAGE_H_
#define INCLUDE_AGV_STATEMACHINE_STATES_STORAGE_H_

class StatesStorage
{

  virtual unsigned int getCurState() const = 0;
};



#endif /* INCLUDE_AGV_STATEMACHINE_STATES_STORAGE_H_ */
