/*
 * Copyright (C) 2018  Love Mowitz
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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "logic-motion.hpp"

TEST_CASE("Speed request should result in positive torque") {
  Motion motion(1.0f, 1.0f, 2400.0f, 500.0f);

  motion.setSpeedRequest(10.0f);
  motion.setLeftWheelSpeed(5.0f);
  motion.setRightWheelSpeed(4.0f);

  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.step();

  REQUIRE(msgTorque.torqueLeft() > 0);
  REQUIRE(msgTorque.torqueRight() > 0);
}

TEST_CASE("Speed request should result in negative torque") {
  Motion motion(1.0f, 1.0f, 2400.0f, 500.0f);

  motion.setSpeedRequest(4.0f);
  motion.setLeftWheelSpeed(10.3f);
  motion.setRightWheelSpeed(10.1f);

  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.step();

  REQUIRE(msgTorque.torqueLeft() < 0);
  REQUIRE(msgTorque.torqueRight() < 0);
}


TEST_CASE("No torque if speed < 5 km/h and decelerating") {
  Motion motion(1.0f, 1.0f, 2400.0f, 500.0f);

  // Standard unit m/s, conversion from km/h
  motion.setSpeedRequest(3.0f / 3.6f);
  motion.setLeftWheelSpeed(4.5f / 3.6f);
  motion.setRightWheelSpeed(4.5f / 3.6f);

  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.step();

  REQUIRE(msgTorque.torqueLeft() == 0);
  REQUIRE(msgTorque.torqueRight() == 0);
}