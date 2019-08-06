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
  float dt = 0.01f;
  float accKp = 300.0f;
  float accKi = 5.0f;
  float torqueLimit = 2400.0f;
  float accILimit = 500.0f;
  float torqueRateLimit = 100.0f;
  Motion motion(dt, accKp, accKi, torqueLimit, accILimit, torqueRateLimit);

  motion.setSpeedRequest(10.0f);
  motion.setGroundSpeedReading(5.0f);

  for (int i = 0; i < 100; i++) {
    motion.step();
  }
  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.getTorque();
  opendlv::proxy::PulseWidthModulationRequest msgBrake = motion.getBrake();

  REQUIRE(msgTorque.torqueLeft() > 0);
  REQUIRE(msgTorque.torqueRight() > 0);
  
  // No braking when torque is non-zero
  REQUIRE(msgBrake.dutyCycleNs() == 0U);
}

TEST_CASE("Speed request should result in negative torque") {
  float dt = 0.01f;
  float accKp = 300.0f;
  float accKi = 5.0f;
  float torqueLimit = 2400.0f;
  float accILimit = 500.0f;
  float torqueRateLimit = 100.0f;
  Motion motion(dt, accKp, accKi, torqueLimit, accILimit, torqueRateLimit);

  motion.setSpeedRequest(4.0f);
  motion.setGroundSpeedReading(10.2f);

  for (int i = 0; i < 100; i++) {
    motion.step();
  }
  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.getTorque();
  opendlv::proxy::PulseWidthModulationRequest msgBrake = motion.getBrake();

  REQUIRE(msgTorque.torqueLeft() < 0);
  REQUIRE(msgTorque.torqueRight() < 0);

  // No braking when torque is non-zero
  REQUIRE(msgBrake.dutyCycleNs() == 0U);
}


TEST_CASE("No torque if speed < 5 km/h and decelerating") {
  float dt = 0.01f;
  float accKp = 300.0f;
  float accKi = 5.0f;
  float torqueLimit = 2400.0f;
  float accILimit = 500.0f;
  float torqueRateLimit = 100.0f;
  Motion motion(dt, accKp, accKi, torqueLimit, accILimit, torqueRateLimit);

  // Standard unit m/s, conversion from km/h
  motion.setSpeedRequest(3.0f / 3.6f);
  motion.setGroundSpeedReading(4.5f/3.6f);

  for (int i = 0; i < 100; i++) {
    motion.step();
  }
  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.getTorque();

  REQUIRE(msgTorque.torqueLeft() == 0);
  REQUIRE(msgTorque.torqueRight() == 0);
}

TEST_CASE("Torque doesn't increase immediately") {
  float dt = 0.01f;
  float accKp = 300.0f;
  float accKi = 5.0f;
  float torqueLimit = 2400.0f;
  float accILimit = 500.0f;
  float torqueRateLimit = 100.0f;
  Motion motion(dt, accKp, accKi, torqueLimit, accILimit, torqueRateLimit);

  // Standard unit m/s, conversion from km/h
  motion.setSpeedRequest(5.0f);
  motion.setGroundSpeedReading(0.0f);

  for (int i = 0; i < 2; i++) {
    motion.step();
  }

  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.getTorque();

  REQUIRE(msgTorque.torqueLeft() < 100);
  REQUIRE(msgTorque.torqueRight() < 100);
}

TEST_CASE("Torque always less than specified limit")
{
  float dt = 0.05f;
  float accKp = 300.0f;
  float accKi = 40.0f;
  float torqueLimit = 400.0f;
  float accILimit = 500.0f;
  float torqueRateLimit = 100.0f;
  Motion motion(dt, accKp, accKi, torqueLimit, accILimit, torqueRateLimit);

  motion.setSpeedRequest(20.0f);
  motion.setGroundSpeedReading(0.0f);

  for (int i = 0; i < 200; i++) {
    motion.step();
  }
  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.getTorque();

  REQUIRE(msgTorque.torqueLeft() <= torqueLimit);
  REQUIRE(msgTorque.torqueRight() <= torqueLimit);
}

TEST_CASE("Brake when speedRequest is <= 0.0f, also torque should be zero")
{
  float dt = 0.01f;
  float accKp = 300.0f;
  float accKi = 40.0f;
  float torqueLimit = 400.0f;
  float accILimit = 500.0f;
  float torqueRateLimit = 100.0f;
  Motion motion(dt, accKp, accKi, torqueLimit, accILimit, torqueRateLimit);

  motion.setSpeedRequest(0.0f);
  motion.setGroundSpeedReading(10.0f);

  for (int i = 0; i < 100; i++) {
    motion.step();
  }
  opendlv::cfsdProxy::TorqueRequestDual msgTorque = motion.getTorque();
  opendlv::proxy::PulseWidthModulationRequest msgBrake = motion.getBrake();
  
  REQUIRE(msgTorque.torqueLeft() == Approx(0.0f).epsilon(0.01f));
  REQUIRE(msgTorque.torqueRight() == Approx(0.0f).epsilon(0.01f));
  REQUIRE(msgBrake.dutyCycleNs() > 0U);
}