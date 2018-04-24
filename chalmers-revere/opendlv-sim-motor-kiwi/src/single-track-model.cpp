/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <iostream>

#include "single-track-model.hpp"

SingleTrackModel::SingleTrackModel() noexcept:
  m_wheelSpeedLeftMutex{}, //Added this
  m_wheelSpeedRightMutex{}, //Added this
  m_wheelSpeedLeft{}, //Added this
  m_wheelSpeedRight{} //Added this
{
}

//Added this
void SingleTrackModel::setWheelSpeedLeft(opendlv::proxy::WheelSpeedRequest const &wheelSpeedRequestLeft) noexcept
{
  std::lock_guard<std::mutex> lock(m_wheelSpeedLeftMutex);
  m_wheelSpeedLeft = wheelSpeedRequestLeft.wheelSpeed();
}

//Added this
void SingleTrackModel::setWheelSpeedRight(opendlv::proxy::WheelSpeedRequest const &wheelSpeedRequestRight) noexcept
{
  std::lock_guard<std::mutex> lock(m_wheelSpeedRightMutex);
  m_wheelSpeedRight = wheelSpeedRequestRight.wheelSpeed();
}

opendlv::sim::KinematicState SingleTrackModel::step(double dt) noexcept
{
  float wheelSpeedLeftCopy;
  float wheelSpeedRightCopy;

  {
    std::lock_guard<std::mutex> lock3(m_wheelSpeedLeftMutex); //Added this
    std::lock_guard<std::mutex> lock4(m_wheelSpeedRightMutex); //Added this
    wheelSpeedLeftCopy = m_wheelSpeedLeft; //Added this
    wheelSpeedRightCopy = m_wheelSpeedRight; //Added this
  }

  //Added this
  double R = 0.12;
  double m_fi = 0.0;
  double m_vx = 0.0;
  double m_vy = 0.0;
  double fi = 0.0;
  double vx = 0.0;
  double vy = 0.0;

  //Kinematics for yawRate
  fi = -((wheelSpeedLeftCopy-wheelSpeedRightCopy)/(2*R));
  m_fi += fi*dt;
  //Kinematics for vx
  vx = (wheelSpeedLeftCopy+wheelSpeedRightCopy)/2*cos(m_fi);
  m_vx += vx*dt;
  //Kinematics for vy
  vy = (wheelSpeedLeftCopy+wheelSpeedRightCopy)/2*sin(m_fi);
  m_vy += vy*dt;

  opendlv::sim::KinematicState kinematicState;

  //Added this
  kinematicState.vx(static_cast<float>(m_vx));
  kinematicState.vy(static_cast<float>(m_vy));
  kinematicState.yawRate(static_cast<float>(m_fi)); 

  return kinematicState;
}
