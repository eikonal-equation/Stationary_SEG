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
* File: main.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: main file.
*
* ==============================================================================
*/

#include "CFMM.hpp"
#include "misc.hpp"
#include "CGradientDescentTracer.hpp"
#include "SimpleFunctions.hpp"
#include "CStationaryObserver.hpp"
#include "CAdversarialPlan.hpp"
#include <chrono>
#include "CStationaryTerrain.hpp"
#include <memory>
#include "PaperExamples.hpp"

using namespace memory;
using namespace std;

int main(int argc, char* argv[]){

  if (argc < 2 ){
    std::cout << "No figure number provided. Running figure 6." << std::endl;
    FigureSix();
    return 0;
  }

  switch(stoi(std::string(argv[1]))) {
      case 3 :
          FigureThree();
          break;
      case 4 :
          FigureFour();
          break;
      case 5 :
          FigureFive();
          break;
      case 6 :
          FigureSix();
          break;
      case 9 :
          FigureNine();
          break;
      case 10 :
          FigureTen();
          break;
      case 11 :
          FigureEleven();
          break;
      case 12 :
          FigureTwelve();
          break;
      default:
        std::cout << "Input not recognized. Number is likely wrong. See README.md" << std::endl;
  }


  return 0;

}
