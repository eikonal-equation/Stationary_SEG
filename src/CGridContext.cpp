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
* File: CGridContext.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CGridContext.hpp
*
* ==============================================================================
*/

#include "CGridContext.hpp"
#include "SimpleFunctions.hpp"
#include "misc.hpp"

using namespace memory;
using namespace std;


CGridContext::CGridContext( std::function<REAL_T(REAL_T, REAL_T)> aCost, std::function<REAL_T(REAL_T, REAL_T)> aSpeed , int aN, REAL_T aPhysMin, REAL_T aPhysMax ){
  assert( aPhysMax >= aPhysMin);
  fMinX = aPhysMin;
  fMinY = aPhysMin;
  fMaxX = aPhysMax;
  fMaxY = aPhysMax;
  fValues = std::make_shared<array2D_t<REAL_T>>(allocate_2d_array<REAL_T>(aN,aN));
  fH = (aPhysMax - aPhysMin)/(aN-1);
  fCost = aCost;
  fSpeed = aSpeed;
}

//Delegating constructor.
CGridContext::CGridContext( int aN,  REAL_T aPhysMin, REAL_T aPhysMax,
  std::function<REAL_T(REAL_T, REAL_T)> aCost, std::function<REAL_T(REAL_T, REAL_T)> aSpeed  ) :
CGridContext( aCost,  aSpeed , aN, aPhysMin, aPhysMax ) {
}


// This writes Speed, Value and Cost on the grid to file.
void CGridContext::writeGridsToFile( string aFilename ) const {
  array2D_t<REAL_T> CostArray = allocate_2d_array<REAL_T>(getGridSizeX(), getGridSizeY());
  array2D_t<REAL_T> SpeedArray = allocate_2d_array<REAL_T>(getGridSizeX(), getGridSizeY());

  for ( int i = 0 ; i < getGridSizeX() ; ++ i){
    for ( int j = 0 ; j < getGridSizeY() ; ++j){
      CostArray[i][j]  = getCost(i,j);
      SpeedArray[i][j]  = getSpeed(i,j);
    }
  }

  io::write_to_file<REAL_T>( aFilename + "Cost" , CostArray , getGridSizeX(), getGridSizeY() );
  io::write_to_file<REAL_T>(  aFilename + "Speed" , SpeedArray ,getGridSizeX(), getGridSizeY());
  io::write_to_file<REAL_T>( aFilename + "Value" , (*fValues), getGridSizeX(), getGridSizeY() );
}

void CGridContext::setSpeed( std::function<REAL_T(REAL_T, REAL_T)> aSpeed ){
  fSpeed = aSpeed;
}

void CGridContext::setCost( std::function<REAL_T(REAL_T, REAL_T)> aCost ){
  fCost = aCost;
}
