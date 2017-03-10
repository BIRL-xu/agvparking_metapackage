/*
 * history_state.h
 *
 *  Created on: Mar 8, 2017
 *      Author: paul
 */

#ifndef INCLUDE_AGV_STATEMACHINE_HISTORY_STATE_H_
#define INCLUDE_AGV_STATEMACHINE_HISTORY_STATE_H_

class HistoryState
{
public:
  HistoryState(unsigned int state):hist_state_(state){}
  ~HistoryState();

  unsigned int getHistoryState() const {return hist_state_;}
  void setHistoryState(unsigned int hist_state) const {this->hist_state_ = hist_state;}

private:
  unsigned int hist_state_;

};



#endif /* INCLUDE_AGV_STATEMACHINE_HISTORY_STATE_H_ */
