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

#ifndef MOTION_H
#define MOTION_H

#include "opendlv-standard-message-set.hpp"
#include "cfsd-extended-message-set.hpp"

#include <mutex>

struct pidObject
{
  float iError;
  const float kp;
  const float ki;
  const float outputLimit;
  const float iLimit;
};

class Motion {
  public:
    Motion(float dt, float accKp, float torqueLimit);
    ~Motion();

  public:
    opendlv::cfsdProxy::TorqueRequestDual step();

    void setGroundSpeedReading(float groundSpeed);
    void setSpeedRequest(float groundSpeed);

  private:
    float calculateTorque(float speedError, float speedReading);


  private:
    float m_dt;
    pidObject m_accPid;
    pidObject m_brakePid;

    // Readings and requests
    float m_groundSpeed;
    float m_speedRequest;

    // System parameters
    struct
    {
      const float m = 217.4f;
      const float gearRatio = 16.0f;
      const float A = 1.14f;
      const float rho = 1.18415f;
      const float Cd = 1.21f;
      const float g = 9.81f;
      const float fr = 0.01f;
      const float Rw = 0.22f;
    } m_param;

    // Message mutexes
    std::mutex m_speedReadingMutex;
    std::mutex m_speedRequestMutex;

    
};
#endif

