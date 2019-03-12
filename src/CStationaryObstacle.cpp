/*
* ==============================================================================
*
*  Copyright (C) 2019  M.A. Gilles
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* ------------------------------------------------------------------------------
*
* The primary purpose in distributing this source code is to enable readers to
* reproduce the numerical  results reported in the manuscript
* "Surveillance-Evasion Games under Uncertainty" by M.A. Gilles and A.
* Vladimirsky. See <https://arxiv.org/abs/1812.10620>.
*
* The code can be found at <https://github.com/eikonal-equation/Stationary_SEG>.
* Please see README.md for instructions on configuring/running this program.
*
* ------------------------------------------------------------------------------
*
* File: CStationaryObstacle.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CStationaryObstacle.hpp
*
* ==============================================================================
*/

#include "CStationaryObstacle.hpp"

CStationaryObstacle::~CStationaryObstacle(){}

bool CStationaryObstacle::isIn( REAL_T aX, REAL_T aY, REAL_T aT){
  if ( aX >= fLeft && aX <= fRight && aY >= fBottom && aY <= fTop ){
    return 1;
  }else{
    return 0;
  }
}

CStationaryObstacle::CStationaryObstacle( REAL_T Left, REAL_T Right, REAL_T Bottom, REAL_T Top ){
  fLeft = Left;
  fRight = Right;
  fBottom = Bottom;
  fTop = Top;
}
