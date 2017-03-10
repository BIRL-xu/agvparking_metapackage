/*
 * orthogonal_states.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: paul
 */
#include "agv_statemachine/orthogonal_states.h"

OrthogonalStates::OrthogonalStates()
{
  std::cout << "Enter Orthogonal states!\n";
}
OrthogonalStates::~OrthogonalStates()
{
  std::cout << "Exit Orthogonal states!\n";
}

Video::Video()
{
  std::cout << "Enter Video states!\n";
}
Video::~Video()
{
  std::cout << "Exit Video states!\n";
}

Music::Music()
{
  std::cout << "Enter Music states!\n";
}
Music::~Music()
{
  std::cout << "Exit Music states!\n";
}

std::string Video::getCurState() const
{
  return std::string("Video");
}

std::string Music::getCurState() const
{
  return std::string("Music");
}
