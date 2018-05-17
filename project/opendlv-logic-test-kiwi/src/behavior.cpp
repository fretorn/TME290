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
#include <cmath>

Behavior::Behavior() noexcept:
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{}
{
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

//Added this
opendlv::proxy::DistanceReading Behavior::getFrontUltrasonic() noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  return m_frontUltrasonicReading;
}

//Added this
opendlv::proxy::DistanceReading Behavior::getRearUltrasonic() noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  return m_rearUltrasonicReading;
}

//Added this
double Behavior::getLeftIr() noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  return convertIrVoltageToDistance(m_leftIrReading.voltage());
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

float globalTime = 0.0f; //Added this
float reverseTime = 11.0f; //Added this

int reverse = 0; 

void Behavior::step(float speed, float front, float rear, float side, float sideWall, float reverseTimeThreshold, float groundSteering, float wallSteering, float rearMin, float reverseSpeed) noexcept
{
  globalTime = globalTime + 1.0f; //Added this
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage());

  float groundSteeringAngle = 0.0f;
  float pedalPosition = speed;

  std::string scenario = ""; 
  
  if (frontDistance < front) {
    reverse = 1;

    //TODO: make it stop reversing if too close to wall.
    //TODO: Make a parameter for reverse speed.
    
  } else if (rearDistance < rear){
    pedalPosition = speed;
  }

  if (frontDistance <front && (leftDistance > side || rightDistance > side)) {
    if (leftDistance > rightDistance) {
      scenario = "turnLeft";
    } else if (rightDistance > leftDistance){
      scenario = "turnRight";
    }
    
  }
  if (scenario == "turnLeft") {
    groundSteeringAngle = groundSteering;
  } else {
    if (scenario == "turnRight") {
      groundSteeringAngle = -groundSteering;
    }
  }

  if (leftDistance < sideWall) {
    groundSteeringAngle = groundSteeringAngle - wallSteering;
  } else if (rightDistance < sideWall) {
    groundSteeringAngle = groundSteeringAngle + wallSteering;
  }

  //Reverse
  if (reverse == 1) {
    reverseTime = reverseTime + 1.0f;
    pedalPosition = -reverseSpeed;
    groundSteeringAngle = -groundSteering; //Added this
  } 
  
  if (reverseTime > reverseTimeThreshold) {
    reverse = 0;
    reverseTime = 0.0f;
  }

  if (rearDistance < rearMin) {
    pedalPosition = speed;
  }

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedalPosition);
    m_pedalPositionRequest = pedalPositionRequest;
  }
}

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = -5.8454*pow(sensorVoltage,3) +36.3658*pow(sensorVoltage,2) -74.3506*sensorVoltage + 56.4574; //Added this
  return distance;
}
