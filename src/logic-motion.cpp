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

#include "cluon-complete.hpp"
#include "logic-motion.hpp"

Motion::Motion(cluon::OD4Session &od4)
  : m_od4{od4}
  , m_leftWheelSpeed{0.0f}
  , m_rightWheelSpeed{0.0f}
  , m_speedRequest{0.0f}

  , m_readingsMutex{}
{
  setUp();
}

Motion::~Motion()
{
  Motion::tearDown();
}

void Motion::setUp()
{
}

void Motion::tearDown()
{
}

void Motion::step()
{
  float leftWheelSpeed;
  float rightWheelSpeed;
  float speedRequest;
  {
    std::lock_guard<std::mutex> lock(m_readingsMutex);

    leftWheelSpeed = m_leftWheelSpeed;
    rightWheelSpeed = m_rightWheelSpeed;
    speedRequest = m_speedRequest;
  }



  cluon::data::TimeStamp sampleTime = cluon::time::now();

  int leftTorque = static_cast<int>(leftWheelSpeed - speedRequest);
  int rightTorque = static_cast<int>(rightWheelSpeed - speedRequest);

  // Send to 2101
  opendlv::cfsdProxy::TorqueRequestDual msgTorque;
  msgTorque.torqueLeft(leftTorque);
  msgTorque.torqueRight(rightTorque);
}

void Motion::setLeftWheelSpeed(float speed)
{
  std::lock_guard<std::mutex> lock(m_readingsMutex);
  m_leftWheelSpeed = speed;
}

void Motion::setRightWheelSpeed(float speed)
{
  std::lock_guard<std::mutex> lock(m_readingsMutex);
  m_rightWheelSpeed = speed;
}

void Motion::setSpeedRequest(float speed)
{
  std::lock_guard<std::mutex> lock(m_readingsMutex);
  m_speedRequest = speed;
}