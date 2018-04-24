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

opendlv::sim::KinematicState SingleTrackModel::step(double dt) noexcept
{
  // double const pedalSpeedGain{0.5};

/*   double const mass{1.0};
  double const momentOfInertiaZ{0.1};
  double const length{0.22};
  double const frontToCog{0.11};
  double const rearToCog{length - frontToCog};
  double const corneringStiffnessFront{1.0};
  double const corneringStiffnessRear{1.0}; */
  
  float wheelSpeedLeftCopy;
  float wheelSpeedRightCopy;
  //float groundSteeringAngleCopy;
  //float pedalPositionCopy;
  {
    
    //std::lock_guard<std::mutex> lock1(m_groundSteeringAngleMutex);
    //std::lock_guard<std::mutex> lock2(m_pedalPositionMutex);
    std::lock_guard<std::mutex> lock3(m_wheelSpeedLeftMutex); //Added this
    std::lock_guard<std::mutex> lock4(m_wheelSpeedRightMutex); //Added this
    wheelSpeedLeftCopy = m_wheelSpeedLeft; //Added this
    wheelSpeedRightCopy = m_wheelSpeedRight; //Added this
    //groundSteeringAngleCopy = m_groundSteeringAngle;
    //pedalPositionCopy = m_pedalPosition;
  }


  // TODO : Write kinematics and use wheelSpeedCopy for the calculations
/*   float R = 0.12f;
  float m_fi = 0.0f;
  float m_vx = 0.0f;
  float m_vy = 0.0f;
  float fi = 0.0f;
  float vx = 0.0f;
  float vy = 0.0f; */
  double R = 0.12;
  double m_fi = 0.0;
  double m_vx = 0.0;
  double m_vy = 0.0;
  double fi = 0.0;
  double vx = 0.0;
  double vy = 0.0;

  // TODO: Kinematics for yawRate
  fi = -((wheelSpeedLeftCopy-wheelSpeedRightCopy)/(2*R));
  m_fi += fi*dt;
  // TODO: Kinematics for vx
  vx = (wheelSpeedLeftCopy+wheelSpeedRightCopy)/2*cos(m_fi);
  m_vx += vx*dt;
  // TODO: Kinematics for vy
  vy = (wheelSpeedLeftCopy+wheelSpeedRightCopy)/2*sin(m_fi);
  m_vy += vy*dt;


/*   m_longitudinalSpeed = pedalPositionCopy * pedalSpeedGain;

  if (std::abs(m_longitudinalSpeed) > 0.01f) {
    double const slipAngleFront = groundSteeringAngleCopy 
      - (m_lateralSpeed + frontToCog * m_yawRate) 
      / std::abs(m_longitudinalSpeed);
    double const slipAngleRear = (rearToCog * m_yawRate - m_lateralSpeed) 
      / std::abs(m_longitudinalSpeed);

    double const lateralSpeedDot = (corneringStiffnessFront * slipAngleFront 
        + corneringStiffnessRear * slipAngleRear)
      / (mass - m_longitudinalSpeed * m_yawRate);

    double const yawRateDot = (frontToCog * corneringStiffnessFront * slipAngleFront 
        - rearToCog * corneringStiffnessRear * slipAngleRear)
      / momentOfInertiaZ;
    
    m_lateralSpeed += lateralSpeedDot * dt;
    m_yawRate += yawRateDot * dt;
  } else {
    m_lateralSpeed = 0.0f;
    m_yawRate = 0.0f;
  } */

  // TODO: Make the calculated wheelspeed part of kinematicState? vx and vy?
  opendlv::sim::KinematicState kinematicState;
  //kinematicState.vx(static_cast<float>(m_longitudinalSpeed));
  //kinematicState.vy(static_cast<float>(m_lateralSpeed));
  //kinematicState.yawRate(static_cast<float>(m_yawRate));
  kinematicState.vx(static_cast<float>(m_vx));
  kinematicState.vy(static_cast<float>(m_vy));
  kinematicState.yawRate(static_cast<float>(m_fi)); //Added this

  return kinematicState;
}
