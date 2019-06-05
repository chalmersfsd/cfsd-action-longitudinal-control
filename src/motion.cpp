/*
 * Copyright (C) 2018  Christian Berger
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

#include "logic-motion.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid") || 
        0 == commandlineArguments.count("freq") ||
        0 == commandlineArguments.count("pGain") ||
        0 == commandlineArguments.count("iGain") ||
        0 == commandlineArguments.count("pLimit") ||
        0 == commandlineArguments.count("iLimit") ||
        0 == commandlineArguments.count("torqueLimit")) {
        std::cerr << argv[0] << "Generates the acceleration requests for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session ID> --freq=<microservice frequency> " <<
                  "--pGain=<Proportional controller gain> -- iGain=<Integration controller gain> " <<
                  "--pLimit=<Proportional feedback limit> --iLimit=<Integration feedback limit> " <<
                  "--torqueLimit=<Torque limit (0-2400)> [--verbose=<Print or not>]" << std::endl;

        std::cerr << "Example: " << argv[0] << "--cid=111 --freq=30 --pGain=1 --iGain=0.5 " <<
             "--pLimit=10 --iLimit=5 --torqueLimit=240 [--verbose]" << std::endl;
        retCode = 1;
    } else {

      // Interface to a running OpenDaVINCI session.  
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
      float FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};
      float pGain{static_cast<float>(std::stof(commandlineArguments["pGain"]))};
      float iGain{static_cast<float>(std::stof(commandlineArguments["iGain"]))};
      float pLimit{static_cast<float>(std::stof(commandlineArguments["pLimit"]))};
      float iLimit{static_cast<float>(std::stof(commandlineArguments["iLimit"]))};
      int torqueLimit{static_cast<int>(std::stoi(commandlineArguments["torqueLimit"]))};
      bool VERBOSE{static_cast<bool>(commandlineArguments.count("verbose"))};

      // 
      if (torqueLimit < 0 || torqueLimit > 2400) {
        std::cerr << "Specified torque limit not between 0 - 2400\nExiting program..." << std::endl;
        return -1;
      } 

      Motion motion(pGain, iGain, pLimit, iLimit, torqueLimit);

      //TODO: Should we use wheelSpeedReadings or filtered groundSpeedReading?
      auto onWheelSpeedReading{[&motion, VERBOSE](cluon::data::Envelope &&envelope)
        {
          uint16_t senderStamp = envelope.senderStamp();
          if (senderStamp == 1904) {
            auto wheelSpeedReading = cluon::extractMessage<opendlv::proxy::WheelSpeedReading>(std::move(envelope));
             motion.setLeftWheelSpeed(wheelSpeedReading.wheelSpeed());
            if (VERBOSE) {
                std::cout << "[ACTION-MOTION] FL wheel speed reading: " << wheelSpeedReading.wheelSpeed() << std::endl;
            }
          } else if (senderStamp == 1903) {
            auto wheelSpeedReading = cluon::extractMessage<opendlv::proxy::WheelSpeedReading>(std::move(envelope));
            motion.setRightWheelSpeed(wheelSpeedReading.wheelSpeed());
            if (VERBOSE) {
              std::cout << "[ACTION-MOTION] FR wheel speed reading: " << wheelSpeedReading.wheelSpeed() << std::endl;
            }
         }
        }};
      od4.dataTrigger(opendlv::proxy::WheelSpeedReading::ID(), onWheelSpeedReading);

      auto onGroundSpeedRequest{[&motion, &od4, VERBOSE](cluon::data::Envelope &&envelope)
        {
          uint16_t senderStamp = envelope.senderStamp();
          if (senderStamp == 2201) {
            auto gsr = cluon::extractMessage<opendlv::proxy::GroundSpeedRequest>(std::move(envelope));
            motion.setSpeedRequest(gsr.groundSpeed());

            if (VERBOSE) {
              std::cout << "[ACTION-MOTION] Groundspeed request: " << gsr.groundSpeed() << std::endl;
            }
          }
        }};
      od4.dataTrigger(opendlv::proxy::GroundSpeedRequest::ID(), onGroundSpeedRequest);


      auto atFrequency{[&motion, &od4, VERBOSE]() -> bool
        {
            // Calculate and send torque request at specified frequency
            opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.step();
            cluon::data::TimeStamp sampleTime = cluon::time::now();
            od4.send(msgTorque, sampleTime, 2101);

            if (VERBOSE) {
              std::cout << "Torque request left: " << msgTorque.torqueLeft() << " | right: " << msgTorque.torqueRight() << std::endl; 
            }
            
            return true;
        }};
      od4.timeTrigger(FREQ, atFrequency);

    }
    return retCode;
}

