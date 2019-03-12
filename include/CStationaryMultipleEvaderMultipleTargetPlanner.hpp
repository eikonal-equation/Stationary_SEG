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
* File: CStationaryMultipleEvaderMultipleTargetPlanner.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: This is an implementation of the CFixedLambdaPlanner class for
* stationary problems with multiple evaders with different targets, sources or
* speed functions. Its role is to build CGridContext, CStationaryTerrain,
* CStationaryObserver, CFMM and CGradientDescentTracer objects. It uses all
* of these classes to compute the pair of probability distribution and set of
* paths which form a Nash Equilibrium.
*
* ==============================================================================
*/


#ifndef CSTATIONARYMULTIPLEEVADERMULTIPLETARGETPLANNER_HPP
#define CSTATIONARYMULTIPLEEVADERMULTIPLETARGETPLANNER_HPP

#include "math.h"

// STL
#include <vector>
#include <string>

// Other file/classes
#include "CFixedLambdaPlanner.hpp"
#include "global_configuration.hpp"
#include "memory_allocations.hpp"
#include "CStationaryObserver.hpp"
#include "CStationaryMultipleEvaderPlanner.hpp"
#include "CGridContext.hpp"
#include "CFMM.hpp"
#include "CGradientDescentTracer.hpp"
#include "CStationaryTerrain.hpp"

// External libraries
#include <Eigen>

class CStationaryMultipleEvaderMultipleTargetPlanner : public CFixedLambdaPlanner
{
private:
  std::vector<std::shared_ptr<CStationaryMultipleEvaderPlanner>> fSinglePlanners;

public:
  /**
   * The destructor
   */
  virtual ~CStationaryMultipleEvaderMultipleTargetPlanner() {};

  /**
   * The Constructor from a vector of CStationaryMultipleEvaderPlanner objects.
   */
   CStationaryMultipleEvaderMultipleTargetPlanner() = default;
  CStationaryMultipleEvaderMultipleTargetPlanner(
    std::vector<std::shared_ptr<CStationaryMultipleEvaderPlanner>> aSinglePlanners
  );

  /**
   * The CStationaryMultipleEvaderMultipleTargetPlanner Constructor.
   * This is the main CStationaryMultipleEvaderMultipleTargetPlanner object constructor.
   * @param aObsLocations a Nx2 std::vector of std::vector containing the locations of the observer, where N is the number of observer locations. For example, initialize to vector<REAL_T> aObsLocations = { { 0.1, 0.5}, {0.4, 0.6}} for two observers at locations (0.1,0.4) and (0.4,0.6).
   * @param aSource a std::vector of std::vector of size Ex2 where E is the number of evaders, containing the (x,y) coordinates of the source of each evader
   * @param aTarget a std::vector of std::vector of size Ex2 where E is the number of evaders, containing the (x,y) coordinates of the target of each evader
   * @param aEvadersWeight a length E std::vector containing the weight of each evader.
   * @param aTerrain a CStationaryTerrain object specifying the obstacles. A std::vector of std::vector of coordinates, defining rectangular obstacles can be passed too, see the CTerrain object constructor.
   * @param aVis function pointer REAL_T(REAL_T,REAL_T,REAL_T,REAL_T) to a observability function which defines the observability of the domain from a fixed an observer position. takes 4 REAL_T inputs: 2 coordinates of the observer and 2 coordinate of the location that is observed. The function outputs a single REAL_T: the observability. Defaults to inverse squared distance: vis = 1/(| x - y|^2 + eps ). Note that all observers have the same observability function but parametrized by different locations.
   * @param aSpeed function pointer REAL_T(REAL_T,REAL_T) to a speed function which defines the speed of the evader at all points in the domain. takes 2 REAL_T inputs: 2 coordinates of the evader. The function outputs a single REAL_T: the speed at that location. Defaults to constant aSpeed = 1.
   * @param aN integer defining the number of grid points in each ShortDirection.  Defaults to 501 (observed to be sufficient to resolve line of sight).
   * @param aPartialCostGrids boolean flag which determines whether the secondary linear equations "v_i" are computed and used to compute the subgradients. Defaults to false (and uses integration instead).
   * @param aMinPhys real number, the lower bound defining the square domain [aMinPhys,aMaxPhys]x[aMinPhys, aMaxPhys]. Defaults to 0.
   * @param aMaxPhys real number, the upper bound defining the square domain [aMinPhys,aMaxPhys]x[aMinPhys, aMaxPhys].  Defaults to 1.
   */
   CStationaryMultipleEvaderMultipleTargetPlanner( std::vector<std::vector<REAL_T>> aObsLocations,
                     std::vector<std::vector<REAL_T>> aEvadersSource,
                     std::vector<std::vector<REAL_T>> aTarget,
                     std::vector<REAL_T> aEvadersWeight,
                     CStationaryTerrain terrain = CStationaryTerrain(),
                     std::function<REAL_T(REAL_T, REAL_T,REAL_T, REAL_T)> aVis = &inverseSquared,
                     std::function<REAL_T(REAL_T, REAL_T)> aSpeed = &constant,
                     int aN = 501,
                     bool aPartialCostGrids = false,
                     REAL_T aMinPhys = 0,
                     REAL_T aMaxPhys = 1 );

 /* Function which computes value and subgradient of objective.
  * @param aLambda (input) a Eigen::VectorXd, the probability of each observer location.
  * @param aValue (output) a real number, the value of the objective function at lambda
  * @param aValue (output) a Eigen::VectorXd, the subgradient of the objective function at lambda.
  */
  virtual void getValueAndSubgradient(  const Eigen::VectorXd aLambda,
    REAL_T& aValue,
    Eigen::VectorXd& aSubgradient) override;

  /* Function which computes value and subgradient of objective of a single observation plan. This is to be used by a Stochastic gradient descent method.
   * @param aLambda (input) a VectorXd of Eigen, the probability of each observer location.
   * @param aValue (output) a real number, the value of the objective function at lambda
   * @param aValue (output) a VectorXd of Eigen, the subgradient of the objective function at lambda.
   * @param aI (input), the index of the planner that is used.
   */
   virtual void getValuesAndSubgradients(  const Eigen::VectorXd aLambda,
     REAL_T& aValue,
     Eigen::VectorXd& aSubgradient,
     int aI = 0) override;

  /* Helper function which returns the number of possible observer locations.
   */
  virtual int getNumberOfObservers() override;

  /* Helper function which returns the number of plans
   */
  virtual int getNumberOfPlans() override;

  /* This saves the current strategy to file. Useful since this object is continually reused in CAdversarialPlan.
   */
  virtual void saveStrategyToFile(std::string filename) override;

  virtual void writeAllToFile(std::string aFilename = "");

};
#endif
