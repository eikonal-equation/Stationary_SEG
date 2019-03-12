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
* File: misc.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: Printing to command line or file functions.
*
* ==============================================================================
*/

#ifndef MISC_HPP
#define MISC_HPP
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "global_configuration.hpp"
#include "memory_allocations.hpp"
#include "Eigen"
namespace io {

//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
// Writes Boost array to file
template <class T>
void write_to_file( std::string aFilename, memory::array2D_t<T> aArray, int aDim0 = 0 , int aDim1 = 0 ){
  aDim0 = aArray.shape()[0];
  aDim1 = aArray.shape()[1];
  std::ofstream out( "output/" + aFilename , std::ios::binary );

  for (int i = 0 ; i < aDim0 ; ++i) {
    for (int j = 0 ; j < aDim1 - 1; ++j) {
      out.write ((char*) &aArray[i][j], sizeof(T) );
    }
    out.write ((char*) &aArray[i][aDim1 -1], sizeof(T) );
  }
}

// Writes std::vector to file.
template <class T>
void write_vector_to_file( std::string aFilename, std::vector<T> aVec ){
  std::ofstream out( "output/" + aFilename  );
  for (int i = 0 ; i < aVec.size()  ; ++i) {
      out.write ((char*) &aVec[i], sizeof(T) );
    }
}


// Prints boost array to command line.
template <class T>
void print(memory::array2D_t<T> aArray, int aDim0, int aDim1){
  for (int i = 0 ; i < aDim0 ; ++i) {
    for (int j = 0 ; j < aDim1 - 1 ; ++j) {
      std::cout << aArray[i][j] <<',';
    }
    std::cout << aArray[i][aDim1 -1];
    std::cout << '\n';
  }
}
}

//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//


#endif
