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
* File: CObserver.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: The abstract Observer class. Owns an abstract terrains which
* define the observability.
*
* ==============================================================================
*/

#ifndef COBSERVER_HPP
#define COBSERVER_HPP
#include <math.h>
#include "CTerrain.hpp"
#include <limits>

class CObserver
{
  public:

    // getters
    virtual REAL_T getObservability( REAL_T aX, REAL_T aY, REAL_T aT = 0 ) const = 0;
    virtual ~CObserver() {};

  protected:

    std::shared_ptr<CTerrain> fTerrain;  /**< Terrain object defining the obstacles */
    std::function<REAL_T(REAL_T, REAL_T,REAL_T,REAL_T)> fUnobstructedObservability;  /**< The observability function */

};



#endif
