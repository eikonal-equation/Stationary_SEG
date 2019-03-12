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
* File: SimpleFunctions.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: Simple functions to be used as observability and speed.
*
* ==============================================================================
*/

#ifndef SIMPLEFUNCTIONS_HPP
#define SIMPLEFUNCTIONS_HPP

inline REAL_T constant(REAL_T x, REAL_T y){
  return 1;
}

inline REAL_T wavy(REAL_T x, REAL_T y){
  return 1 + 0.5*cos((REAL_T) x*10*3.141592653589793238463)*cos((REAL_T) y*10*3.141592653589793238463) ;
}

inline REAL_T exponential(REAL_T posx, REAL_T posy, REAL_T x, REAL_T y){
  return exp( - 20*( ( posx - x)*( posx - x) +  ( posy - y)*( posy - y) ) );
}

inline REAL_T inverseSquared(REAL_T posx, REAL_T posy, REAL_T x, REAL_T y){
  return 1/( (( posx - x)*( posx - x) +  ( posy - y)*( posy - y)) + 0.1);
}

inline REAL_T smallInverseSquared(REAL_T posx, REAL_T posy, REAL_T x, REAL_T y){
  return 1/( 30*(( posx - x)*( posx - x) +  ( posy - y)*( posy - y)) + 0.1);
}


#endif
