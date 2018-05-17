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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "behavior.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
    std::cerr << argv[0] << " tests the Kiwi platform by sending actuation commands and reacting to sensor input." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --freq=<Integration frequency> --cid=<OpenDaVINCI session> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --freq=10 --cid=111" << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);
    float speed = std::stof(commandlineArguments["speed"]);
    float front = std::stof(commandlineArguments["front"]);
    float rear = std::stof(commandlineArguments["rear"]);
    float goalDistanceToWall = std::stof(commandlineArguments["goalDistanceToWall"]);
    float sideWall = std::stof(commandlineArguments["sideWall"]);
    float reverseTimeThreshold = std::stof(commandlineArguments["reverseTimeThreshold"]);
    float groundSteering = std::stof(commandlineArguments["groundSteering"]);
    float wallSteering = std::stof(commandlineArguments["wallSteering"]);
    float rearMin = std::stof(commandlineArguments["rearMin"]);
    float reverseSpeed = std::stof(commandlineArguments["reverseSpeed"]);
    float Kp_side = std::stof(commandlineArguments["Kp_side"]);
    float sideDistanceForStraightReverse = std::stof(commandlineArguments["sideDistanceForStraightReverse"]);
    float frontDistance45 = std::stof(commandlineArguments["frontDistance45"]);
    float sideDistance45 = std::stof(commandlineArguments["sideDistance45"]);
    float forwardTimeAfterReverseLimit = std::stof(commandlineArguments["forwardTimeAfterReverseLimit"]);
    float addAngleAfterReverse = std::stof(commandlineArguments["addAngleAfterReverse"]);
    

    Behavior behavior;

    auto onDistanceReading{[&behavior](cluon::data::Envelope &&envelope)
      {
        auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == 0) {
          behavior.setFrontUltrasonic(distanceReading);
        } else {
          behavior.setRearUltrasonic(distanceReading);
        }
      }};
    auto onVoltageReading{[&behavior](cluon::data::Envelope &&envelope)
      {
        auto voltageReading = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == 0) {
          behavior.setLeftIr(voltageReading);
        } else {
          behavior.setRightIr(voltageReading);
        }
      }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
    od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);

    //In here it is decided what the car should do.
    auto atFrequency{[&VERBOSE, &behavior, &od4, &speed, &front, &rear, 
    &goalDistanceToWall, &sideWall, &reverseTimeThreshold, &groundSteering, 
    &wallSteering, &rearMin, &reverseSpeed, &FREQ, &Kp_side, 
    &sideDistanceForStraightReverse, &frontDistance45, &sideDistance45,
    &forwardTimeAfterReverseLimit, &addAngleAfterReverse]() -> bool
      {
        behavior.step(speed, front, rear, goalDistanceToWall, sideWall, 
        reverseTimeThreshold, groundSteering, wallSteering, rearMin, 
        reverseSpeed, FREQ, Kp_side, sideDistanceForStraightReverse, frontDistance45, sideDistance45,
        forwardTimeAfterReverseLimit, addAngleAfterReverse);
        auto groundSteeringAngleRequest = behavior.getGroundSteeringAngle();
        auto pedalPositionRequest = behavior.getPedalPositionRequest();
        auto frontUltrasonicReading = behavior.getFrontUltrasonic();
        auto rearUltrasonicReading = behavior.getRearUltrasonic();
        auto leftIrReading = behavior.getLeftIr();

        cluon::data::TimeStamp sampleTime;
        od4.send(groundSteeringAngleRequest, sampleTime, 0);
        od4.send(pedalPositionRequest, sampleTime, 0);
        if (VERBOSE) {
          std::cout << "Steer " << std::setw(6) << groundSteeringAngleRequest.groundSteering()
            << " Pedal " << std::setw(6) << pedalPositionRequest.position() 
            << " Front " <<  std::setw(6) << frontUltrasonicReading.distance()
            << " Rear " <<  std::setw(6) << rearUltrasonicReading.distance()
            << " Left " <<  std::setw(6) << leftIrReading
            // << " Right " << rightDistance
            << std::endl;
        }

        return true;
      }};

    od4.timeTrigger(FREQ, atFrequency);
  }
  return retCode;
}
