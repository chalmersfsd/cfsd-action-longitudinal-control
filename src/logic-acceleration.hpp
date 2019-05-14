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

#ifndef ACCELERATION_H
#define ACCELERATION_H

#include "opendlv-standard-message-set.hpp"

enum asState {
    AS_OFF,
    AS_READY, 
    AS_DRIVING, 
    AS_FINISHED,
    AS_EMERGENCY,
    AS_MANUAL
 };

class Acceleration {
  public:
    Acceleration(cluon::OD4Session &od4);
    ~Acceleration();

  public:
    void step();

    void setAsState(asState state);
    void setLeftWheelSpeed(float speed);
    void setRightWheelSpeed(float speed);

  private:
    void setUp();
    void tearDown();


  private:
    cluon::OD4Session &m_od4;
    asState m_asState;
    float m_leftWheelSpeed;
    float m_rightWheelSpeed;

    
};
#endif

