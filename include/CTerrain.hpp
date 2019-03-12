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
* File: CTerrain.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: The Abstract terrain class.  It is only responsible for figuring
* out whether any given point in the domain is contained is contained in any of
* the abstract obstacles.
*
* ==============================================================================
*/

#ifndef CTERRAIN_HPP
#define CTERRAIN_HPP
#include <vector>
#include "CObstacle.hpp"
#include "global_configuration.hpp"

class CTerrain{
  public:
    /**
    * Returns true if (aX,aY) is inside any of the obstacles
    */
    virtual REAL_T isNotInObstacles( REAL_T aX, REAL_T aY, REAL_T aT = 0 );
    // desctructor
    virtual ~CTerrain();
    // constructors
    CTerrain() = default;
    CTerrain( std::vector<std::shared_ptr<CObstacle>> aObstacles );

  protected:
    std::vector<std::shared_ptr<CObstacle>> fObstacles; /**< pointer to vector of obstacles */
    int fNumObstacles;
};

#endif
