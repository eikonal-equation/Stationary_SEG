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
* File: CStationaryMultipleEvaderMultipleTargetPlanner.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CStationaryMultipleEvaderMultipleTargetPlanner.hpp
*
* ==============================================================================
*/

#include "CStationaryMultipleEvaderMultipleTargetPlanner.hpp"
#include "CStationaryMultipleEvaderPlanner.hpp"
#include "SimpleFunctions.hpp"
#include "misc.hpp"
#include <math.h>
#include <iostream>
#include <functional>
using namespace Eigen;
using namespace memory;
using namespace std;
using namespace std::placeholders;


CStationaryMultipleEvaderMultipleTargetPlanner::CStationaryMultipleEvaderMultipleTargetPlanner(
  std::vector<std::shared_ptr<CStationaryMultipleEvaderPlanner>> aSinglePlanners
){
  fSinglePlanners = aSinglePlanners;
}


CStationaryMultipleEvaderMultipleTargetPlanner::CStationaryMultipleEvaderMultipleTargetPlanner(
    std::vector<std::vector<REAL_T>> aLocations,
    std::vector<std::vector<REAL_T>> aEvadersSource,
    std::vector<std::vector<REAL_T>> aTarget,
    std::vector<REAL_T> aEvadersWeight,
    CStationaryTerrain aTerrain ,
    std::function<REAL_T(REAL_T, REAL_T,REAL_T, REAL_T)> aVis,
    std::function<REAL_T(REAL_T, REAL_T)> aSpeed,
    int aN,
    bool aPartialCostGrids,
    REAL_T aMinPhys,
    REAL_T aMaxPhys )
{
  fSinglePlanners.resize(aTarget.size());
  vector<vector<REAL_T>> SingleSource = {{0,0}};
  vector<REAL_T> SingleWeight = {0};

  for (int i = 0 ; i < aTarget.size(); ++i){
    SingleSource[0] = aEvadersSource[i];
    SingleWeight[0] = aEvadersWeight[i];
    fSinglePlanners[i] = make_shared<CStationaryMultipleEvaderPlanner>(aLocations, SingleSource, aTarget[i], SingleWeight, aTerrain, aVis, aSpeed, aN, aPartialCostGrids, aMinPhys, aMaxPhys);
  }
}

void CStationaryMultipleEvaderMultipleTargetPlanner::saveStrategyToFile(std::string aFilename){
  for ( int i = 0 ; i < fSinglePlanners.size(); ++i){
    fSinglePlanners[i]->saveStrategyToFile(aFilename+"P"+to_string(i));
  }
}


void CStationaryMultipleEvaderMultipleTargetPlanner::writeAllToFile(std::string aFilename){
  for ( int i = 0 ; i < fSinglePlanners.size(); ++i){
    fSinglePlanners[i]->writeAllToFile(aFilename+"P"+to_string(i));
  }
}


void CStationaryMultipleEvaderMultipleTargetPlanner::getValueAndSubgradient(
  const Eigen::VectorXd aLambda,
  REAL_T& aValue,
  Eigen::VectorXd& aSubgradient){

  REAL_T singleValue;
  Eigen::VectorXd singleSubgradient(aLambda.size());

  aValue = 0;
  aSubgradient =  VectorXd::Constant(aLambda.size(),0);

  for ( int i = 0 ; i < fSinglePlanners.size(); ++i){
    fSinglePlanners[i]->getValueAndSubgradient(aLambda, singleValue, singleSubgradient);
    aValue+= singleValue;
    aSubgradient += singleSubgradient;
  }

}

void CStationaryMultipleEvaderMultipleTargetPlanner::getValuesAndSubgradients(
  const Eigen::VectorXd aLambda,
  REAL_T& aValue,
  Eigen::VectorXd& aSubgradient,
  int aI ){
  cout << "Using evader: " << aI << endl;
  fSinglePlanners[aI]->getValueAndSubgradient(aLambda, aValue, aSubgradient);
}


int CStationaryMultipleEvaderMultipleTargetPlanner::getNumberOfObservers()
{
  return fSinglePlanners[0]->getNumberOfObservers();
}

/* Helper function which returns the number of possible observer locations.
 */
int CStationaryMultipleEvaderMultipleTargetPlanner::getNumberOfPlans(){
  return fSinglePlanners.size();
}
