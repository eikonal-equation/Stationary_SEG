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
* File: memory_allocations.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: Thin wrapper around boost::multi_array.
*
* ==============================================================================
*/

#ifndef memory_allocations_h
#define memory_allocations_h
#include "boost/multi_array.hpp"

//------------------------------------------------------------------------------------------------//
//
//------------------------------------------------------------------------------------------------//
namespace memory {


template<class T>
using array2D_t =  boost::multi_array<T, 2>;

template <class T>
array2D_t<T> allocate_2d_array(
    const int n1, const int n2
) {
  array2D_t<T> array(boost::extents[n1][n2]);
  return array;
}

template <class T>
void resizeArray2D(
    array2D_t<T> &array , const int n1, const int n2
) {
  array.resize(boost::extents[n1][n2]);
  return;
}



//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

} // namespace memory

//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//


#endif /* memory_allocations_h */
