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

Motion::Motion(float accKp, float accKi, float torqueLimit, float accILimit)
  : m_accPid{0.0f, accKp, accKi, 2.0f*torqueLimit, accILimit}
  , m_brakePid{0.0f, accKp, accKi, torqueLimit, accILimit}
  , m_leftWheelSpeed{0.0f}
  , m_rightWheelSpeed{0.0f}
  , m_speedRequest{0.0f}

  , m_leftWheelSpeedMutex{}
  , m_rightWheelSpeedMutex{}
  , m_speedRequestMutex{}
{
  std::cout << "Setting up longitudinal controller..." << std::endl;
  std::cout << " Done." << std::endl;
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
  int torqueLeft = static_cast<int>(torque * 0.5f);
  int torqueRight = static_cast<int>(torque * 0.5f);



  // ----------------------- RETURN CORRECT MESSAGE TYPE ----------------------
  opendlv::cfsdProxy::TorqueRequestDual msgTorque;
  msgTorque.torqueLeft(torqueLeft);
  msgTorque.torqueRight(torqueRight);

  return msgTorque;
}

float Motion::calculateTorque(float error)
{
  // ---------------------------- CALCULATE TORQUE ----------------------------

  // Only accumulate error when small enough
  // to avoid too much wind up
  if (error < 5.0f) {
    m_accPid.iError += error;
  }

  // Limit integral feedback
  float iFeedback = m_accPid.iError * m_accPid.kp;
  iFeedback = iFeedback < m_accPid.iLimit ? iFeedback : m_accPid.iLimit;

  // Limit total torque output
  float torque = error * m_accPid.kp + iFeedback;
  torque = torque < m_accPid.outputLimit ? torque : m_accPid.outputLimit;

  return torque;
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