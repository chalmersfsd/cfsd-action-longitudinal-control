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

#include "logic-acceleration.hpp"
#include <iostream>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
        std::cerr << argv[0] << "Generates the acceleration requests for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --freq=<Microservice frequency> [--verbose=<Verbose or not>]"
        << std::endl;
        std::cerr << "Example: " << argv[0] << "--cid=111 --freq=2 [--verbose]" << std::endl;
        retCode = 1;
    } else {

        // Interface to a running OpenDaVINCI session.  
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        float FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};
        bool VERBOSE{static_cast<bool>(commandlineArguments.count("verbose"))};

        Acceleration acceleration(od4);

        auto onSwitchStateReading{[&acceleration, VERBOSE](cluon::data::Envelope &&envelope)
        {
            uint16_t senderStamp = envelope.senderStamp();
            if (senderStamp == 1401) {
                auto state = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                acceleration.setAsState(static_cast<asState>(state.state()));
                if (VERBOSE) {
                    std::cout << "[LOGIC-ACCELERATION] AS state reading: " << state.state() << std::endl;
                }
            }
        }};
        od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);

        //TODO: Should we use wheelSpeedReadings or filtered groundSpeedReading?
        auto onWheelSpeedReading{[&acceleration, VERBOSE](cluon::data::Envelope &&envelope)
        {
          uint16_t senderStamp = envelope.senderStamp();
          if (senderStamp == 1904) {
            auto wheelSpeedReading = cluon::extractMessage<opendlv::proxy::WheelSpeedReading>(std::move(envelope));
            acceleration.setLeftWheelSpeed(wheelSpeedReading.wheelSpeed());
            if (VERBOSE) {
              std::cout << "[LOGIC-ACCELERATION] FL wheel speed reading: " << wheelSpeedReading.wheelSpeed() << std::endl;
            }
          } else if (senderStamp == 1903) {
            auto wheelSpeedReading = cluon::extractMessage<opendlv::proxy::WheelSpeedReading>(std::move(envelope));
            acceleration.setRightWheelSpeed(wheelSpeedReading.wheelSpeed());
            if (VERBOSE) {
              std::cout << "[LOGIC-ACCELERATION] FR wheel speed reading: " << wheelSpeedReading.wheelSpeed() << std::endl;
            }
          }
        }};
        od4.dataTrigger(opendlv::proxy::WheelSpeedReading::ID(), onWheelSpeedReading);

        auto atFrequency{[&acceleration, &VERBOSE]() -> bool
        {
            acceleration.step();
            return true;
        }};

        od4.timeTrigger(FREQ, atFrequency);


    }
    return retCode;
}

