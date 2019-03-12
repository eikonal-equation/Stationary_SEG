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
* File: global_configuration.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: Defines constants and types used in the code.
*
* ==============================================================================
*/

#ifndef __GlobalConfig_h
#define __GlobalConfig_h

//-- STL -----------------------------------------------------------------------------------------//
#include <assert.h>
#include <functional>
#include <limits>
//------------------------------------------------------------------------------------------------//
//-- Function overrides
//------------------------------------------------------------------------------------------------//


//------------------------------------------------------------------------------------------------//
//-- Floating point type
//------------------------------------------------------------------------------------------------//
#define REAL_T double
#define STATUS_T unsigned short int


//------------------------------------------------------------------------------------------------//
//-- Constants
//------------------------------------------------------------------------------------------------//
#define INF std::numeric_limits<REAL_T>::max()
#define PI 3.141592653589793
#define SQRT2 1.41421356237309504880168872420969807


//------------------------------------------------------------------------------------------------//
//-- CFMM gridpoint this.fStatus
//------------------------------------------------------------------------------------------------//
#define FAR 0
#define CONSIDERED 1
#define ACCEPTED 2

//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

#endif // header file endif
