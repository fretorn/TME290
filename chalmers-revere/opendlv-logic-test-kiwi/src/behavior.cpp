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
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_wheelSpeedRequestLeft{}, //Added this
  m_wheelSpeedRequestRight{}, //Added this
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  m_wheelSpeedRequestLeftMutex{}, //Added this
  m_wheelSpeedRequestRightMutex{} //Added this
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


void Behavior::step() noexcept
{
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
/*   double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage()); */


  // TODO: Does this set wheelSpeed to zero??
  float wheelSpeedLeft = 0.5f; //Added this
  float wheelSpeedRight = 0.5f; //Added this

  if (frontDistance < 0.3f) {
    wheelSpeedLeft = 0.0f;
    wheelSpeedRight = 0.0f;
  } else {
    if (rearDistance < 0.2f) {
      wheelSpeedLeft = 0.5f;
      wheelSpeedRight = 0.5f;
    }
  }

  /* if (leftDistance < rightDistance) {
    if (leftDistance < 0.1f) {
      wheelSpeedLeft = 0.5f;
    }
  } else {
    if (rightDistance < 0.1f) {
      wheelSpeedRight = 0.5f;
    }
  } */

  // TODO: I think there should be some more logic here. Maybe to control both wheelSpeed 0 and 1?

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

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = (2.5 - sensorVoltage) / 0.07;
  return distance;
}
