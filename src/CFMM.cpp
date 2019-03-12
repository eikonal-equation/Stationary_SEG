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
* File: CFMM.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CFMM.hpp
*
* ==============================================================================
*/

#include "CFMM.hpp"
#include <math.h>       /* fabs */
#include <iostream>
using namespace std;
using namespace memory;

// 5-point stencil
int stencil[4][2] = {{1,0},{-1, 0}, {0, 1}, {0, -1}};


CFMM::CFMM(shared_ptr<CGridContext> aPrimary){
  // Initialize properties and allocate arrays.
  fPrimary = aPrimary;
  fStatus = std::make_unique<array2D_t<STATUS_T>>(allocate_2d_array<STATUS_T>(fPrimary->getGridSizeX(),fPrimary->getGridSizeY()));
  fHeapPointers = std::make_unique<array2D_t<handle_t>>(allocate_2d_array<handle_t>(fPrimary->getGridSizeX(),fPrimary->getGridSizeY()));
  fNumSecondary = 0;
}



CFMM::CFMM(shared_ptr<CGridContext> aPrimary, shared_ptr<vector<CGridContext>> aSecondary){
  // Initialize properties and allocate arrays.
  fPrimary = aPrimary;
  fStatus = std::make_unique<array2D_t<STATUS_T>>(allocate_2d_array<STATUS_T>(fPrimary->getGridSizeX(),fPrimary->getGridSizeY()));
  fHeapPointers = std::make_unique<array2D_t<handle_t>>(allocate_2d_array<handle_t>(fPrimary->getGridSizeX(),fPrimary->getGridSizeY()));
  fSecondary = aSecondary;
  fNumSecondary = aSecondary->size();
}


void CFMM::march(
  REAL_T aTargetX, // physical coordinates
  REAL_T aTargetY
) {
    assert((fPrimary->getMinX() <= aTargetX) && \
           (fPrimary->getMaxX() >=  aTargetX) && \
           (fPrimary->getMinY() <= aTargetY) && \
           (fPrimary->getMaxY() >=  aTargetY) );

    int TargetI = fPrimary->xPhysicalToGrid(aTargetX);
    int TargetJ = fPrimary->yPhysicalToGrid(aTargetY);

    CFMMHeap_t CFMMHeap;
    initializeCFMM( CFMMHeap, TargetI, TargetJ);

    while ( ! CFMMHeap.empty() ) {
        CHeapGP current_GP = CFMMHeap.top();
        CFMMHeap.pop();
        (*fStatus)[current_GP.fI][current_GP.fJ] = ACCEPTED;
        updateNeighbors( CFMMHeap, current_GP.fI, current_GP.fJ);
    }

}


inline
void CFMM::initializeCFMM(
    CFMMHeap_t& aCFMMHeap,
    int aTargetX,
    int aTargetY
) {

    // All grid points are set to FAR.
    for (int i = 0; i < fPrimary->getGridSizeX(); ++ i) {
        for (int j = 0; j < fPrimary->getGridSizeY(); ++ j) {
            fPrimary->setValue(i, j, INF);
            (*fStatus)[i][j]      = FAR;
        }
    }

    // Except the Target, for which we know the solution.
    fPrimary->setValue(aTargetX, aTargetY, 0);
    for ( int k = 0 ; k < fNumSecondary; ++k){
      (*fSecondary)[k].setValue(aTargetX, aTargetY, 0);
    }
    // push Target in Heap.
    (*fStatus)[aTargetX][aTargetY] = ACCEPTED;
    aCFMMHeap.push(CHeapGP(aTargetX, aTargetY, 0));

}

inline
void CFMM::updateNeighbors(
    CFMMHeap_t& aCFMMHeap,
    const  int aCurrent_i,
    const  int aCurrent_j
) {

    // Iterate over neighbors.
    for (int k = 0; k < 4; ++ k) {
        int nbr_i = aCurrent_i + stencil[k][0];
        int nbr_j = aCurrent_j + stencil[k][1];

        if (inDomain(nbr_i,nbr_j)) {
            // If accepted nothing to do, otherwise, try to update
            if ((*fStatus)[nbr_i][nbr_j] != ACCEPTED) {
                const bool gp_was_updated = updateGP(nbr_i, nbr_j)  ;
                // If it was already considered and was updated then update heap
                if ((*fStatus)[nbr_i][nbr_j] == CONSIDERED) {
                    if (gp_was_updated) {
                      aCFMMHeap.update( (*fHeapPointers)[nbr_i][nbr_j],
                          CHeapGP(nbr_i, nbr_j, fPrimary->getValue(nbr_i,nbr_j) ));
                    }
                } else {
                  // Else add to heap
                    (*fStatus)[nbr_i][nbr_j] = CONSIDERED;
                    (*fHeapPointers)[nbr_i][nbr_j]  =
                      aCFMMHeap.push( CHeapGP(nbr_i, nbr_j, fPrimary->getValue(nbr_i,nbr_j) ) );
                }
            }
        }
    }
}

inline
bool CFMM::inDomain ( const int aI, const int aJ ) const
{
  if ( aI >= 0 && aI < fPrimary->getGridSizeX() && aJ >=0 && aJ < fPrimary->getGridSizeY()){
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------------------------//
//-- Helper functions
//------------------------------------------------------------------------------------------------//

inline
void CFMM::smallerHneighbor(const int aI, const int aJ, std::vector<REAL_T>& aVal) const {
  int bestI;
    if (aI == 0) {
        bestI =  aI + 1;
    } else if (aI == fPrimary->getGridSizeX() - 1 ) {
        bestI =  aI - 1;
    } else if ( fPrimary->getValue(aI-1, aJ) <  fPrimary->getValue(aI+1, aJ) ){
        bestI =  aI - 1;
    } else {
        bestI = aI + 1;
    }

    aVal[0] = fPrimary->getValue(bestI, aJ);
    for ( int k = 0 ; k < fNumSecondary ; ++k){
      aVal[k+1] = (*fSecondary)[k].getValue(bestI, aJ);
    }
    return;
}


inline
void CFMM::smallerVneighbor(const int aI, const int aJ, std::vector<REAL_T>& aVal) const {
    int bestJ;
    if (aJ == 0) {
      bestJ = aJ + 1;
    } else if (aJ == fPrimary->getGridSizeY() - 1) {
        bestJ =  aJ - 1;
    } else if (fPrimary->getValue(aI,aJ - 1) < fPrimary->getValue(aI, aJ + 1) ) {
        bestJ = aJ - 1;
    } else{
        bestJ = aJ + 1;
    }

    aVal[0] = fPrimary->getValue(aI, bestJ);
    for ( int k = 0 ; k < fNumSecondary ; ++k){
      aVal[k+1] = (*fSecondary)[k].getValue(aI, bestJ);
    }
    return;
}

//------------------------------------------------------------------------------------------------//
//-- Compute functions
//------------------------------------------------------------------------------------------------//

inline
void CFMM::computeFromTwoNeighbors(const std::vector<REAL_T>& aF, const std::vector<REAL_T>& aVal1, const std::vector<REAL_T>& aVal2, std::vector<REAL_T>& aNewVal) {
    assert(!isnan(aF[0]));

    if(aF[0] == 0) {
      for ( int k = 0 ; k < fNumSecondary + 1 ; ++ k){
        aNewVal[k] = INF;
      }
    }

    const REAL_T h = fPrimary->getH();

    vector<REAL_T> diff(fNumSecondary + 1);
    vector<REAL_T> rat1(fNumSecondary + 1);

    for ( int k = 0 ; k < fNumSecondary + 1 ; ++ k){
      diff[k] = aVal1[k] - aVal2[k];
      rat1[k] = h/aF[k];
    }


    if (fabs(diff[0]) >= rat1[0]) {//upwinding condition won't be satisfied. Do one sided update
        if (aVal1[0] < aVal2[0]){
          for ( int k = 0 ; k < fNumSecondary + 1 ; ++ k ){
            aNewVal[k] = aVal1[k] + rat1[k];
          }
        } else {
          for ( int k = 0 ; k < fNumSecondary + 1 ; ++ k ){
            aNewVal[k] = aVal2[k] + rat1[k];
          }
        }
    } else { // Else Do two sided update

      REAL_T rat = SQRT2*rat1[0];
      REAL_T discriminant = (rat - diff[0])*(rat + diff[0]);
      assert(discriminant >= 0);

      aNewVal[0] = .5*( (aVal1[0] + aVal2[0]) + sqrt(discriminant) );

      for ( int k = 1 ; k < fNumSecondary + 1 ; ++ k ){
        aNewVal[k] = (  - rat1[0] * rat1[k]
                  + aVal1[k] * (aVal1[0] - aNewVal[0] )
                  + aVal2[k] * (aVal2[0] - aNewVal[0] ) )
                /( aVal1[0] - aNewVal[0] + aVal2[0] - aNewVal[0]);
      }

    }
    return;
}

//------------------------------------------------------------------------------------------------//
//-- Update gridpoint
//------------------------------------------------------------------------------------------------//

inline
bool CFMM::updateGP(const int aI, const int aJ) {

    auto valH = vector<REAL_T>(fNumSecondary + 1);
    auto valV = vector<REAL_T>(fNumSecondary + 1);
    auto new_value = vector<REAL_T>(fNumSecondary + 1);
    auto f = vector<REAL_T>(fNumSecondary + 1);

    // Compute smaller vertical and horizontal neighbors.
    smallerHneighbor(aI,aJ,valH);
    smallerVneighbor(aI,aJ,valV);

    f[0] = fPrimary->getSpeed(aI,aJ)/fPrimary->getCost(aI,aJ); // f is really speed/cost
    if ( isnan(f[0]) ){
      cout << " At (i,j) = " << aI << " ," << aJ << endl;
      cout << " Cost = " << fPrimary->getCost(aI,aJ) << endl;
      cout << " Speed = " << fPrimary->getSpeed(aI,aJ) << endl;
      assert(!isnan(f[0]));
    };

    for ( int k = 0 ; k < fNumSecondary ; ++k){
      f[k+1] = (*fSecondary)[k].getSpeed(aI,aJ)/(*fSecondary)[k].getCost(aI,aJ); //
    }

    // If speed is 0, then value has to be inf there.
    if (f[0] == 0) {
        fPrimary->setValue(aI,aJ,INF);
        for ( int k = 0 ; k < fNumSecondary ; ++k){
          (*fSecondary)[k].setValue(aI,aJ,INF);
        }
        return false;
    }

     computeFromTwoNeighbors(f, valH, valV, new_value);


    if (new_value[0] < fPrimary->getValue(aI,aJ)) {
        fPrimary->setValue(aI, aJ, new_value[0]);
        for ( int k = 0 ; k < fNumSecondary ; ++k){
          (*fSecondary)[k].setValue(aI,aJ, new_value[k+1]);
        }
        return true;
    } else {
        return false;
    }
}
