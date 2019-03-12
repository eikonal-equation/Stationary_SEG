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
* File: CStationaryObserver.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CStationaryObserver.hpp
*
* ==============================================================================
*/

#include "CStationaryObserver.hpp"
#include "CFMM.hpp"

using namespace memory;
using namespace std;
using namespace std::placeholders;

const double CStationaryObserver::ObservabilityInShadows = 0.1;


CStationaryObserver::CStationaryObserver( REAL_T aPosX, REAL_T aPosY, \
  std::shared_ptr<CStationaryTerrain> aTerrain, \
  std::function<REAL_T(REAL_T, REAL_T,REAL_T,REAL_T)> aObs, \
  int aN, \
  REAL_T aPhysMin, \
  REAL_T aPhysMax )
{
  fPosX = aPosX;
  fPosY = aPosY;
  fTerrain = aTerrain;
  computeShadows(aN, aPhysMin, aPhysMax);
  fUnobstructedObservability = aObs;
}


void CStationaryObserver::computeShadows( int aN, REAL_T aPhysMin, REAL_T aPhysMax ){

  fShadows = make_shared<CGridContext>(aN, aPhysMin, aPhysMax);
  CFMM FMMobject = CFMM(fShadows);
  FMMobject.march(fPosX, fPosY);

  array2D_t<REAL_T> Unobstructed =   memory::allocate_2d_array<REAL_T>(fShadows->getGridSizeX(), fShadows->getGridSizeY());

  // Compute unobstructed distance
  for ( int i = 0 ; i < fShadows->getGridSizeX(); ++ i ){
    for ( int j = 0 ; j < fShadows->getGridSizeY(); ++ j ){
      Unobstructed[i][j] = fShadows->getValue(i,j);
    }
  }

  // Compute obstructed distance
  std::function<REAL_T(REAL_T, REAL_T)> notInsideObstacles = \
            std::bind( &CStationaryTerrain::isNotInObstacles, fTerrain, _1, _2, 0);
  fShadows->setSpeed( notInsideObstacles);
  FMMobject.march(fPosX, fPosY);

  // If obstructed > unobstructed, then the observer can't see there

  for ( int i = 0 ; i < fShadows->getGridSizeX(); ++ i ){
    for ( int j = 0 ; j < fShadows->getGridSizeY(); ++ j ){
      if ( fShadows->getValue(i,j) > Unobstructed[i][j] + fShadows->getH()/10 ){
        fShadows->setValue(i,j,0);
      }
      else{
        fShadows->setValue(i,j,1);
      }
    }
  }

}
