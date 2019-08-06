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

Motion::Motion(float dt, float accKp, float accKi, float torqueLimit, float accILimit, float torqueRateLimit)
  : m_dt{dt}
  , m_prevTorque{0.0f}
  , m_torqueRateLimit{torqueRateLimit}
  , m_torqueLeft{0U}
  , m_torqueRight{0U}
  , m_stop{false}
  , m_brakeDuty{0U}
  , m_accPid{0.0f, accKp, accKi, torqueLimit, accILimit}
  , m_brakePid{0.0f, accKp, accKi, torqueLimit, accILimit}
  , m_groundSpeed{0.0f}
  , m_speedRequest{0.0f}

  , m_speedReadingMutex{}
  , m_speedRequestMutex{}
{
  std::cout << "Setting up longitudinal controller..." << std::endl;
  std::cout << " Done." << std::endl;
}

Motion::~Motion() 
{
}

void Motion::step()
{
  // Make a safe copy of data
  float speedReading, speedRequest;
  {
    std::lock_guard<std::mutex> lock1(m_speedReadingMutex);
    std::lock_guard<std::mutex> lock2(m_speedRequestMutex);

    speedReading = m_groundSpeed;
    speedRequest = m_speedRequest;
  }


  float torque;
  if (m_stop) {
    torque = 0.0f;
    m_brakeDuty = 50000U;
  } else {
    float speedError = speedRequest - speedReading;
    torque = calculateTorque(speedError);
  
    if (torque - m_prevTorque > m_torqueRateLimit * m_dt) {
      torque = m_prevTorque + m_torqueRateLimit * m_dt;
    } else if (torque - m_prevTorque < -m_torqueRateLimit * m_dt) {
      torque = m_prevTorque - m_torqueRateLimit * m_dt;
    }
    m_prevTorque = torque;
    m_brakeDuty = 0U;
  }


  // Check the torque if the speed is below 5 km/h, 
  // important for regenerative braking
  if (speedReading < 5.0f / 3.6f && torque < 0.0f){
    torque = 0.0f;
  }

  // Torque distribution
  m_torqueLeft = static_cast<int>(torque);
  m_torqueRight = static_cast<int>(torque);
}

float Motion::calculateTorque(const float error)
{
  // ---------------------------- CALCULATE TORQUE ----------------------------

  // Only accumulate error when small enough
  // to avoid too much wind up
  if (std::abs(error) < 5.0f) {
    m_accPid.iError += error * m_dt;
  }

  // Limit integral feedback
  if (m_accPid.iError > m_accPid.iLimit) {
    m_accPid.iError = m_accPid.iLimit;
  } else if (m_accPid.iError < 0.0f) {
    m_accPid.iError = 0.0f;
  }
  
  float pFeedback = error * m_accPid.kp;
  float iFeedback = m_accPid.iError * m_accPid.ki;

  // Limit total torque output
  float torque = pFeedback + iFeedback;
  torque = torque > m_accPid.outputLimit ? m_accPid.outputLimit : torque;

  return torque;
}

// ################################# GETTERS ##################################
opendlv::cfsdProxy::TorqueRequestDual Motion::getTorque()
{
  opendlv::cfsdProxy::TorqueRequestDual torque;
  torque.torqueLeft(m_torqueLeft);
  torque.torqueRight(m_torqueRight);
  return torque;
}

opendlv::proxy::PulseWidthModulationRequest Motion::getBrake()
{
  opendlv::proxy::PulseWidthModulationRequest brakeReq;
  brakeReq.dutyCycleNs(m_brakeDuty);
  return brakeReq;
}

// ################################# SETTERS ##################################
void Motion::setGroundSpeedReading(float groundSpeed)
{
  std::lock_guard<std::mutex> lock(m_speedReadingMutex);
  m_groundSpeed = groundSpeed;
}

void Motion::setSpeedRequest(float speed)
{
  std::lock_guard<std::mutex> lock(m_speedRequestMutex);
  m_speedRequest = speed;
  m_stop = speed <= 0.0f ? true : false;
}