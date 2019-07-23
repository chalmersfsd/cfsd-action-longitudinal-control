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

Motion::Motion(float dt, float accKp, float torqueLimit)
  : m_dt{dt}
  , m_accPid{0.0f, accKp, 0.0f, 2.0f*torqueLimit, 0.0f}
  , m_brakePid{0.0f, accKp, 0.0f, torqueLimit, 0.0f}
  , m_groundSpeed{0.0f}
  , m_speedRequest{0.0f}
  , m_param{}

  , m_speedReadingMutex{}
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
    std::lock_guard<std::mutex> lock1(m_speedReadingMutex);
    std::lock_guard<std::mutex> lock2(m_speedRequestMutex);

    speedReading = m_groundSpeed;
    speedRequest = m_speedRequest;
  }

  float speedError = speedRequest - speedReading;
  float torque = calculateTorque(speedError, speedReading);

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

float Motion::calculateTorque(const float speedError, const float speedReading)
{
  // ---------------------------- CALCULATE TORQUE ----------------------------
  // Feedforward based on system resistances
  float force = m_param.m * m_param.g * m_param.fr; // Roll resistance
  force = speedError > 0.0f ? force : 0.0f;
  force += 0.5f * m_param.Cd * m_param.A * m_param.rho * std::pow(speedReading, 2.0f); // air resistance
  float ffTorque = force * m_param.Rw / m_param.gearRatio * 100.0f; // convert to motor torque in cNm

  // Proportionall gain based on error
  float pTorque = speedError * m_accPid.kp;

  // Torque
  float torque = pTorque + ffTorque;
  torque = torque > m_accPid.outputLimit ? m_accPid.outputLimit : torque;

  return torque;
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
}