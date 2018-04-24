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

#include "behavior.hpp"

Behavior::Behavior() noexcept:
  m_wheelSpeedRequestLeft{}, //Added this
  m_wheelSpeedRequestRight{}, //Added this
  m_wheelSpeedRequestLeftMutex{}, //Added this
  m_wheelSpeedRequestRightMutex{} //Added this
{
}

//Added this
opendlv::proxy::WheelSpeedRequest Behavior::getWheelSpeedRequestLeft() noexcept
{
  std::lock_guard<std::mutex> lock(m_wheelSpeedRequestLeftMutex);
  return m_wheelSpeedRequestLeft;
}
//Added this
opendlv::proxy::WheelSpeedRequest Behavior::getWheelSpeedRequestRight() noexcept
{
  std::lock_guard<std::mutex> lock(m_wheelSpeedRequestRightMutex);
  return m_wheelSpeedRequestRight;
}

void Behavior::step() noexcept
{
  {
  }
  float wheelSpeedLeft = 0.0f; //Added this
  float wheelSpeedRight = 0.0f; //Added this

  {
    std::lock_guard<std::mutex> lock1(m_wheelSpeedRequestLeftMutex); //Added this
    std::lock_guard<std::mutex> lock2(m_wheelSpeedRequestRightMutex); //Added this

    //Added this
    opendlv::proxy::WheelSpeedRequest wheelSpeedRequestLeft;
    wheelSpeedRequestLeft.wheelSpeed(wheelSpeedLeft);
    m_wheelSpeedRequestLeft = wheelSpeedRequestLeft;
    //Added this
    opendlv::proxy::WheelSpeedRequest wheelSpeedRequestRight;
    wheelSpeedRequestRight.wheelSpeed(wheelSpeedRight);
    m_wheelSpeedRequestRight = wheelSpeedRequestRight;
  }
}