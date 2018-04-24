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



    auto atFrequency{[&VERBOSE, &behavior, &od4]() -> bool
      {
        float vL = 0.0f;
        float vR = 0.0f;
        //double R = 0.12; //[m] Radius of robot
        float v0 = 0.5f; //[m/s] Initial speed
        float t1 = 3.0f; //[s]
        float t2 = 10.0f; //[s]
        float dt = 0.01f; //[s]
        float t = 0.0f; //[s] Current time
        t += dt; 
        
        //Conditions for wheel speed
        if (t <= t1) {
          vL = 0;
          vR = v0*(t/t1);
        } else {
          if (t <= t2) {
            vL = v0*((t-t1)/t2);
            vR = v0;
          }
        }

        behavior.step();
        auto groundSteeringAngleRequest = behavior.getGroundSteeringAngle();
        auto pedalPositionRequest = behavior.getPedalPositionRequest();
        //auto wheelSpeedRquest = behavior.getWheelSpeedRequest(); //Added this

        //float leftWheelSpeedValue = 0.1f; //Added this
        //float rightWheelSpeedValue = 0.1f; //Added this

        //Added this
        opendlv::proxy::WheelSpeedRequest wheelSpeedRequestLeft;
        wheelSpeedRequestLeft.wheelSpeed(vL);
        opendlv::proxy::WheelSpeedRequest wheelSpeedRequestRight;
        wheelSpeedRequestRight.wheelSpeed(vR);

        cluon::data::TimeStamp sampleTime;
        od4.send(groundSteeringAngleRequest, sampleTime, 0);
        od4.send(pedalPositionRequest, sampleTime, 0);
        od4.send(wheelSpeedRequestLeft, sampleTime, 0); //Added this
        od4.send(wheelSpeedRequestRight, sampleTime, 1); //Added this
        //od4.send(wheelSpeedRequest, sampleTime, 0); //Added this
        if (VERBOSE) {
          std::cout << "Ground steering angle is " << groundSteeringAngleRequest.groundSteering()
            << " and pedal position is " << pedalPositionRequest.position()
            << "aaaand wheel speed left is " << wheelSpeedRequestLeft.wheelSpeed()
            << "aaaand wheel speed right is " << wheelSpeedRequestRight.wheelSpeed() << std::endl; //Added this
        }

        return true;
      }};

    od4.timeTrigger(FREQ, atFrequency);
  }
  return retCode;
}
