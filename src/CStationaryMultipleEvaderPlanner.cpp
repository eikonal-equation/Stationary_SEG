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
* File: CStationaryMultipleEvaderPlanner.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CStationaryMultipleEvaderPlanner.hpp
*
* ==============================================================================
*/

#include "CStationaryMultipleEvaderPlanner.hpp"
#include "SimpleFunctions.hpp"
#include <iostream>
#include "misc.hpp"
#include <math.h>
#include <functional>
#include <Eigen>
using namespace Eigen;
using namespace memory;
using namespace std;
using namespace std::placeholders;

CStationaryMultipleEvaderPlanner::~CStationaryMultipleEvaderPlanner() {}

CStationaryMultipleEvaderPlanner::CStationaryMultipleEvaderPlanner(
    std::vector<std::vector<REAL_T>> aLocations,
    std::vector<REAL_T> aSource,
    std::vector<REAL_T> aTarget,
    CStationaryTerrain aTerrain ,
    std::function<REAL_T(REAL_T, REAL_T,REAL_T, REAL_T)> aVis,
    std::function<REAL_T(REAL_T, REAL_T)> aSpeed,
    int aN,
    bool aPartialCostGrids,
    REAL_T aMinPhys,
    REAL_T aMaxPhys )
{

  // Initialize source,target and observer locations.
  fEvadersSource = {aSource};
  fEvadersWeight = {1};
  fTarget = {aTarget};
  fNumObservers = aLocations.size();
  fObservers = make_shared<std::vector<CStationaryObserver>>(fNumObservers);

  if (aPartialCostGrids){
    fSingleObsGrid = make_shared<std::vector<CGridContext>>(fNumObservers);
  } else{ // else make empty vector
    fSingleObsGrid = make_shared<std::vector<CGridContext>>(0);
  }
  for ( int k = 0 ; k < fNumObservers ; ++k){
    (*fObservers)[k] = CStationaryObserver( aLocations[k][0], aLocations[k][1], make_shared<CStationaryTerrain>( aTerrain) ,aVis, aN);
    if (aPartialCostGrids){
      function<REAL_T(REAL_T, REAL_T)> SingleCost = bind( &CStationaryObserver::getObservability, &(*fObservers)[k] , _1, _2, 0);
      (*fSingleObsGrid)[k] = CGridContext(SingleCost, aSpeed, aN, aMinPhys, aMaxPhys);
    }
  }

  // Initialize the Expected value grid, FMM and fTracer Objects.
  fExpectedObsGrid = make_shared<CGridContext>();
  (*fExpectedObsGrid) = CGridContext(&constant, aSpeed, aN, aMinPhys, aMaxPhys);
  if (aPartialCostGrids){
    fCFMMobject =  make_shared<CFMM>( fExpectedObsGrid, fSingleObsGrid);
  } else{
    fCFMMobject =  make_shared<CFMM>( fExpectedObsGrid);
  }

  fTracer.resize(fEvadersSource.size());
  for (int i = 0 ; i < fEvadersSource.size(); ++i){
    fTracer[i] = make_shared<CGradientDescentTracer>( fExpectedObsGrid, fEvadersSource[i][0], fEvadersSource[i][1], fTarget[0], fTarget[1]);
  }
}


CStationaryMultipleEvaderPlanner::CStationaryMultipleEvaderPlanner(
    std::vector<std::vector<REAL_T>> aLocations,
    std::vector<std::vector<REAL_T>> aEvadersSource,
    std::vector<REAL_T> aTarget,
    std::vector<REAL_T> aEvadersWeight,
    CStationaryTerrain aTerrain ,
    std::function<REAL_T(REAL_T, REAL_T,REAL_T, REAL_T)> aVis,
    std::function<REAL_T(REAL_T, REAL_T)> aSpeed,
    int aN,
    bool aPartialCostGrids,
    REAL_T aMinPhys,
    REAL_T aMaxPhys )
{

  // Initialize source, target and observer locations.
  fEvadersSource = aEvadersSource;
  fEvadersWeight = aEvadersWeight;
  fTarget = aTarget;
  fNumObservers = aLocations.size();
  fObservers = make_shared<std::vector<CStationaryObserver>>(fNumObservers);
  // Initialize Observer locations objects, compute shadow zones.
  if (aPartialCostGrids){
    fSingleObsGrid = make_shared<std::vector<CGridContext>>(fNumObservers);
  } else{ // else make empty vector
    fSingleObsGrid = make_shared<std::vector<CGridContext>>(0);
  }
  for ( int k = 0 ; k < fNumObservers ; ++k){
    (*fObservers)[k] = CStationaryObserver( aLocations[k][0], aLocations[k][1], make_shared<CStationaryTerrain>( aTerrain) ,aVis, aN);
    if (aPartialCostGrids){
      function<REAL_T(REAL_T, REAL_T)> SingleCost = bind( &CStationaryObserver::getObservability, &(*fObservers)[k] , _1, _2, 0);
      (*fSingleObsGrid)[k] = CGridContext(SingleCost, aSpeed, aN, aMinPhys, aMaxPhys);
    }
  }

  // Initialize the Expected value grid, FMM and fTracer Objects.
  fExpectedObsGrid = make_shared<CGridContext>();
  (*fExpectedObsGrid) = CGridContext(&constant, aSpeed, aN, aMinPhys, aMaxPhys);
  if (aPartialCostGrids){
    fCFMMobject =  make_shared<CFMM>( fExpectedObsGrid, fSingleObsGrid);
  } else{
    fCFMMobject =  make_shared<CFMM>( fExpectedObsGrid);
  }

  fTracer.resize(fEvadersSource.size());
  for (int i = 0 ; i < fEvadersSource.size(); ++i){
    fTracer[i] = make_shared<CGradientDescentTracer>( fExpectedObsGrid, fEvadersSource[i][0], fEvadersSource[i][1], fTarget[0], fTarget[1]);
  }

}


REAL_T CStationaryMultipleEvaderPlanner::getExpectedObservability( VectorXd aLambda, REAL_T aX , REAL_T aY){
  REAL_T cost = 0;
  for ( int k = 0 ; k < fNumObservers; ++k ){
      if (aLambda[k] != 0 ){ // this is meant to avoid 0*INF = nan
        cost += (*fObservers)[k].getObservability(aX, aY)*aLambda[k];
        if (isnan(cost)){
          cout << "vis: " << (*fObservers)[k].getObservability(aX, aY) << "; prob "<< aLambda[k] << endl;
          assert(!isnan(cost));
        }
      }
  }
  return cost;
}


void CStationaryMultipleEvaderPlanner::writeAllToFile(string aFilename){
  // Write Expected and single observability grids to file.
  fExpectedObsGrid->writeGridsToFile(aFilename + "Combined");
  for ( int k = 0 ; k < fSingleObsGrid->size(); ++k){
    (*fSingleObsGrid)[k].writeGridsToFile(aFilename +"Obs" + std::to_string(k));
  }

  // Write Path to File
  for ( int i = 0 ; i < fEvadersSource.size(); ++i){
    fTracer[i]->printOptimalPathToFile(aFilename+"E"+to_string(i)+"OptPath");
  }

  io::write_vector_to_file(aFilename+"Target", fTarget);
  vector<REAL_T> tempVec;
  for ( int k = 0 ; k < fEvadersSource.size(); ++k){
    tempVec.push_back(fEvadersSource[k][0]);
    tempVec.push_back(fEvadersSource[k][1]);
  }
  io::write_vector_to_file(aFilename+"Sources", tempVec);

  tempVec.clear();
  for ( int k = 0 ; k < fObservers->size(); ++k){
    tempVec.push_back((*fObservers)[k].getPosX());
    tempVec.push_back((*fObservers)[k].getPosY());
  }
  io::write_vector_to_file(aFilename+"Observers", tempVec);

}

void CStationaryMultipleEvaderPlanner::saveStrategyToFile(std::string aFilename){
  for ( int i = 0 ; i < fEvadersSource.size(); ++i){
    fTracer[i]->printOptimalPathToFile(aFilename+"E"+to_string(i));
  }
}


void CStationaryMultipleEvaderPlanner::getValueAndSubgradient(
  const Eigen::VectorXd aLambda,
  REAL_T& aValue,
  Eigen::VectorXd& aSubgradient){

  // Update Cost function.
  std::function<REAL_T(REAL_T, REAL_T)> CombinedCost =
              std::bind( &CStationaryMultipleEvaderPlanner::getExpectedObservability, this, aLambda,
                _1, _2);
  fExpectedObsGrid->setCost(CombinedCost);

  // Run FMM.
  fCFMMobject->march( fTarget[0], fTarget[1]);

  // Find values
  aValue = 0;
  vector<vector<int>> SourceLogical(fEvadersSource.size(), vector<int>(2));
  for (int i = 0 ; i < fEvadersSource.size(); ++i ){
    int xSource = fExpectedObsGrid->xPhysicalToGrid(fEvadersSource[i][0]);
    int ySource = fExpectedObsGrid->yPhysicalToGrid(fEvadersSource[i][1]);
    aValue -= fEvadersWeight[i]*fExpectedObsGrid->getValue( xSource, ySource);
  }


  // Value function is the solution of Eikonal evaluated at the source.
  // Subgradient obtained by tracing paths.
  aSubgradient =  VectorXd::Constant(fNumObservers,0);

  for (int j = 0 ; j < fEvadersSource.size(); ++j ){
    Path path = fTracer[j]->getOptimalPath();
    for ( int i = 0 ; i < path.size() - 1; ++ i ){
      REAL_T diffx = (path[i+1].x - path[i].x);
      REAL_T diffy = (path[i+1].y - path[i].y);
      REAL_T linelength = sqrt(diffx*diffx + diffy*diffy);
      for ( int k = 0 ; k < fNumObservers ; ++k ){
        aSubgradient[k] -=  fEvadersWeight[j]*linelength*(*fObservers)[k].getObservability(path[i].x, path[i].y)/fExpectedObsGrid->getSpeedPhysical(path[i].x, path[i].y);
      }
    }
    path.clear();
  }
}

int CStationaryMultipleEvaderPlanner::getNumberOfObservers(){
  return fNumObservers;
}
