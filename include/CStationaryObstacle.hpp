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
* File: CStationaryObstacle.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: The StationaryObstacle class. Depends on the CObstacle class.
* Currently only supports grid aligned rectangles. It is only responsible for
% figuring out whether any given point in the domain is contained in the
* obstacle. The idea is to have children shape classes, each having a isIn(x,y)
* which returns true if (x,y) is in the obstacle so that more general shapes of
* obstacles are acceptable.
*
* ==============================================================================
*/



#ifndef CSTATIONARYOBSTACLE_HPP
#define CSTATIONARYOBSTACLE_HPP
#include "global_configuration.hpp"
#include "CObstacle.hpp"
class CStationaryObstacle : public CObstacle {
  public:

    REAL_T fLeft, fRight, fTop, fBottom; /**< The position of the obstacle : [fLeft,fRight]x[fTop,fBottom]  */

    /**
    * Returns true if (aX,aY) is inside the obstacle
    */
    virtual bool isIn(REAL_T aX, REAL_T aY, REAL_T aT = 0) override;

    //Constructors
    CStationaryObstacle() = default;
    CStationaryObstacle( REAL_T Left, REAL_T Right, REAL_T Bottom,  REAL_T Top );

    virtual ~CStationaryObstacle();

};

#endif
