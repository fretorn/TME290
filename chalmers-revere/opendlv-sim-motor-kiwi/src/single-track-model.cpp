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
  m_groundSteeringAngleMutex{},
  m_pedalPositionMutex{},
  m_wheelSpeedLeftMutex{}, //Added this
  m_wheelSpeedRightMutex{}, //Added this
  m_longitudinalSpeed{0.0f},
  m_lateralSpeed{0.0f},
  m_yawRate{0.0f},
  m_groundSteeringAngle{0.0f},
  m_pedalPosition{0.0f},
  m_wheelSpeedLeft{}, //Added this
  m_wheelSpeedRight{}, //Added this
  m_fi{0.0f},
  m_vx{0.0f},
  m_vy{0.0f}
{
}

void SingleTrackModel::setGroundSteeringAngle(opendlv::proxy::GroundSteeringRequest const &groundSteeringAngle) noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleMutex);
  m_groundSteeringAngle = groundSteeringAngle.groundSteering();
}

void SingleTrackModel::setPedalPosition(opendlv::proxy::PedalPositionRequest const &pedalPosition) noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionMutex);
  m_pedalPosition = pedalPosition.position();
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
    std::lock_guard<std::mutex> lock1(m_wheelSpeedLeftMutex); //Added this
    std::lock_guard<std::mutex> lock2(m_wheelSpeedRightMutex); //Added this
    wheelSpeedLeftCopy = m_wheelSpeedLeft; //Added this
    wheelSpeedRightCopy = m_wheelSpeedRight; //Added this
  }

  //Added this
  double R = 0.12*dt/dt;
  double fi = 0.0;

  //Kinematics for yawRate
  fi = -( (wheelSpeedLeftCopy - wheelSpeedRightCopy) / (2 * R) ) + 0.1;

/*   m_vx = m_vx + wheelSpeedLeftCopy * dt;
  m_vy = m_vy + wheelSpeedRightCopy * dt; */

  opendlv::sim::KinematicState kinematicState;

  //Added this
  kinematicState.vx(static_cast<float>(wheelSpeedLeftCopy));
  kinematicState.vy(static_cast<float>(wheelSpeedRightCopy));
  kinematicState.yawRate(static_cast<float>(fi)); 
/*   kinematicState.vx(static_cast<float>(m_vx));
  kinematicState.vy(static_cast<float>(m_vy)); */
  //kinematicState.yawRate(static_cast<float>(fi/fi)); 

  return kinematicState;
}
