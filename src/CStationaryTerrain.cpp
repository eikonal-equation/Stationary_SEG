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
* File: CStationaryTerrain.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CStationaryTerrain.hpp
*
* ==============================================================================
*/

#include "CStationaryTerrain.hpp"
using namespace std;
CStationaryTerrain::~CStationaryTerrain() {}

CStationaryTerrain::CStationaryTerrain(){
  fNumObstacles = 0;
}

CStationaryTerrain::CStationaryTerrain( vector<vector<REAL_T>> aSquareObstacles)
{
  fObstacles.resize(aSquareObstacles.size());
  fNumObstacles = aSquareObstacles.size();
  for ( int i = 0 ; i < fNumObstacles; ++i){
    fObstacles[i] = make_shared<CStationaryObstacle>(aSquareObstacles[i][0],aSquareObstacles[i][1],aSquareObstacles[i][2],aSquareObstacles[i][3] );
  }
}

CStationaryTerrain::CStationaryTerrain( std::vector<std::shared_ptr<CObstacle>> aObstacles )
{
  fObstacles = aObstacles;
  fNumObstacles = aObstacles.size();
}
