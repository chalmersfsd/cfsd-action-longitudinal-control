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

Motion::Motion()
  : m_leftWheelSpeed{0.0f}
  , m_rightWheelSpeed{0.0f}
  , m_speedRequest{0.0f}
  , m_pGain{}

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
  // Calculate constant P gain based on model
  // TODO: see if PI controller needed / better
  const float gearRatio = 16.0f;
  const float mass = 217.4f;
  const float wheelRadius = 0.22f;
  m_pGain = mass * wheelRadius / gearRatio * 100.0f;

  std::cout << "Setting up longitudinal controller" << std::endl;
}

void Motion::tearDown()
{
}

opendlv::cfsdProxy::TorqueRequestDual Motion::step()
{
  // ------------ CALCULATE TORQUE ---------------

  // Copy data to avoid conflict
  float speedReading, speedRequest;
  {
    std::lock_guard<std::mutex> lock(m_readingsMutex);

    speedReading = (m_leftWheelSpeed + m_rightWheelSpeed) / 2.0f;
    speedRequest = m_speedRequest;
  }
  
  float speedError = speedRequest - speedReading;
  float torque = speedError * m_pGain; // In [cNm]

  // Check the torque if the speed is below 5 km/h, important for regenerative braking
  // TODO: Check if there already exists a guard for this in the rear node
  if (speedReading < 5.0f / 3.6f && torque < 0.0f){
    torque = 0.0f;
  }

  // Torque distribution
  int torqueLeft = static_cast<int>(torque * 0.5f);
  int torqueRight = static_cast<int>(torque * 0.5f);


  // ------------ RETURN CORRECT MESSAGE TYPE ---------------
  opendlv::cfsdProxy::TorqueRequestDual msgTorque;
  msgTorque.torqueLeft(torqueLeft);
  msgTorque.torqueRight(torqueRight);

  return msgTorque;
}




// Mutex protected setters
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