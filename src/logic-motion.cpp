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

Motion::Motion(float pGain, float iGain, float pLimit, float iLimit, int torqueLimit)
  : m_pGain{pGain}
  , m_iGain{iGain}
  , m_pLimit{pLimit}
  , m_iLimit{iLimit}
  , m_torqueLimit{torqueLimit}
  , m_leftWheelSpeed{0.0f}
  , m_rightWheelSpeed{0.0f}
  , m_speedRequest{0.0f}
  , m_accelerationToTorqueFactor{}
  , m_iError{0.0f}

  , m_leftWheelSpeedMutex{}
  , m_rightWheelSpeedMutex{}
  , m_speedRequestMutex{}
{
  std::cout << "Setting up longitudinal controller..." << std::endl;

  // Calculate acceleration to torque conversion factor
  const float gearRatio = 16.0f;
  const float mass = 217.4f;
  const float wheelRadius = 0.22f;
  
  // Converts acceleration to [cNm]
  m_accelerationToTorqueFactor = mass * wheelRadius / gearRatio * 100.0f;
}

Motion::~Motion() 
{
}

opendlv::cfsdProxy::TorqueRequestDual Motion::step()
{
  // Make a safe copy of data
  float speedReading, speedRequest;
  {
    std::lock_guard<std::mutex> lock1(m_leftWheelSpeedMutex);
    std::lock_guard<std::mutex> lock2(m_rightWheelSpeedMutex);
    std::lock_guard<std::mutex> lock3(m_speedRequestMutex);

    speedReading = (m_leftWheelSpeed + m_rightWheelSpeed) / 2.0f;
    speedRequest = m_speedRequest;
  }

  float speedError = speedRequest - speedReading;
  float torque = calculateTorque(speedError);

  // Check the torque if the speed is below 5 km/h, 
  // important for regenerative braking
  if (speedReading < 5.0f / 3.6f && torque < 0.0f){
    torque = 0.0f;
  }

  // Torque distribution and limit
  int torqueLeft = std::min(static_cast<int>(torque * 0.5f), m_torqueLimit);
  int torqueRight = std::min(static_cast<int>(torque * 0.5f), m_torqueLimit);



  // ------------ RETURN CORRECT MESSAGE TYPE ---------------
  opendlv::cfsdProxy::TorqueRequestDual msgTorque;
  msgTorque.torqueLeft(torqueLeft);
  msgTorque.torqueRight(torqueRight);

  return msgTorque;
}

float Motion::calculateTorque(float error)
{
  // ------------ CALCULATE TORQUE ---------------

  // Only accumulate error when small enough
  // to avoid too much wind up
  if (error < 3.0f) {
    m_iError += error;
  }

  // Limit PI feedback
  float pFeedback = error * m_pGain;
  pFeedback = pFeedback < m_pLimit ? pFeedback : m_pLimit;

  float iFeedback = m_iError * m_iGain;
  iFeedback = iFeedback < m_iLimit ? iFeedback : m_iLimit;

  float acceleration = pFeedback + iFeedback;

  // Convert acceleration to torque and return
  return acceleration * m_accelerationToTorqueFactor;
}


// ################################# SETTERS ##################################
void Motion::setLeftWheelSpeed(float speed)
{
  std::lock_guard<std::mutex> lock(m_leftWheelSpeedMutex);
  m_leftWheelSpeed = speed;
}

void Motion::setRightWheelSpeed(float speed)
{
  std::lock_guard<std::mutex> lock(m_rightWheelSpeedMutex);
  m_rightWheelSpeed = speed;
}

void Motion::setSpeedRequest(float speed)
{
  std::lock_guard<std::mutex> lock(m_speedRequestMutex);
  m_speedRequest = speed;
}