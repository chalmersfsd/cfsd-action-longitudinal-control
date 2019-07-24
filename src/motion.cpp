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
        0 == commandlineArguments.count("accKp") ||
        0 == commandlineArguments.count("torqueLimit")) {
        std::cerr << argv[0] << "Generates the acceleration requests for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session ID> --freq=<microservice frequency> " <<
                  "--accKp=<Proportional controller gain> " <<
                  "--torqueLimit=<Torque limit (0-2400)>" <<
                  "[--verbose=<Print or not>]" << std::endl;
        std::cerr << "Example: " << argv[0] << "--cid=111 --freq=30 --accKp=1 " <<
             "--torqueLimit=1200 [--verbose]" << std::endl;
        retCode = 1;
    } else {

      // Interface to a running OpenDaVINCI session.  
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
      float FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};
      float accKp{static_cast<float>(std::stof(commandlineArguments["accKp"]))};
      float torqueLimit{static_cast<float>(std::stof(commandlineArguments["torqueLimit"]))};
      bool VERBOSE{static_cast<bool>(commandlineArguments.count("verbose"))};
      float DT{1.0f/FREQ};

      // Check if torque limit is within bounds
      if (torqueLimit < 0 || torqueLimit > 2400) {
        std::cerr << "Specified torque limit not between 0 - 2400\nExiting program..." << std::endl;
        retCode = 2;
        return retCode;
      } 

      Motion motion(DT, accKp, torqueLimit);

      auto onGroundSpeedReading{[&motion, VERBOSE](cluon::data::Envelope &&envelope) 
      {
        uint16_t senderStamp = envelope.senderStamp();
        if (senderStamp == 3000) {
          auto gsr = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(envelope));

          motion.setGroundSpeedReading(gsr.groundSpeed());

          if (VERBOSE) {
            std::cout << "[LYNX-VIEWER] GroundSpeedReading: " << gsr.groundSpeed() << std::endl;
          }
        }
      }};
      od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(), onGroundSpeedReading);

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

          // Send out in standard message set too because of problem with
          // exporting messages from extended message set
          opendlv::proxy::TorqueRequest stdMsgTorque;
          stdMsgTorque.torque(msgTorque.torqueLeft());
          od4.send(stdMsgTorque, sampleTime, 2102);

          stdMsgTorque.torque(msgTorque.torqueRight());
          od4.send(stdMsgTorque, sampleTime, 2103);

          if (VERBOSE) {
            std::cout << "Torque request left: " << msgTorque.torqueLeft() << " | right: " << msgTorque.torqueRight() << std::endl; 
          }
            
          return true;
        }};
      od4.timeTrigger(FREQ, atFrequency);

    }
    return retCode;
}

