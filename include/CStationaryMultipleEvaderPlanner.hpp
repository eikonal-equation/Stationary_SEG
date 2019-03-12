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
* File: CStationaryMultipleEvaderPlanner.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: This is an implementation of the CFixedLambdaPlanner class for
* stationary problems with multiple evader sharing the same source and speed
* functions, or simply a single evader. Its role is to build CGridContext,
* CStationaryTerrain, CStationaryObserver, CFMM and CGradientDescentTracer
* objects. It uses all of these classes to compute the pair of probability
* distribution and set of path which form a Nash Equilibrium.
*
* ==============================================================================
*/

#ifndef CSTATIONARYMULTIPLEEVADERPLANNER_HPP
#define CSTATIONARYMULTIPLEEVADERPLANNER_HPP

#include "math.h"

// STL
#include <vector>
#include <string>

// Other file/classes
#include "CFixedLambdaPlanner.hpp"
#include "global_configuration.hpp"
#include "memory_allocations.hpp"
#include "CStationaryObserver.hpp"
#include "CGridContext.hpp"
#include "CFMM.hpp"
#include "CGradientDescentTracer.hpp"
#include "CStationaryTerrain.hpp"
#include <Eigen>

class CStationaryMultipleEvaderPlanner : public CFixedLambdaPlanner
{


  public:
    // default c-tor/d-tor
    virtual ~CStationaryMultipleEvaderPlanner();
    CStationaryMultipleEvaderPlanner() = default;

    /**
     * The CAdversarialPlan Constructor.
     * This is the main CStationaryMultipleEvaderPlanner object constructor.
     * @param aObsLocations a Nx2 std::vector of std::vector containing the locations of the observer, where N is the number of observer locations. For example, initialize to vector<REAL_T> aObsLocations = { { 0.1, 0.5}, {0.4, 0.6}} for two observers at locations (0.1,0.4) and (0.4,0.6).
     * @param aSource a std::vector of std::vector of size Ex2 where E is the number of evaders, containing the (x,y) coordinates of the source of each evader
     * @param aTarget a length 2 std::vector containing the location of the target. All evaders share the same target.
     * @param aEvadersWeight a length E std::vector containing the weight of each evader.
     * @param aTerrain a CStationaryTerrain object specifying the obstacles. A std::vector of std::vector of coordinates, defining rectangular obstacles can be passed too, see the CTerrain object constructor.
     * @param aVis function pointer REAL_T(REAL_T,REAL_T,REAL_T,REAL_T) to a observability function which defines the observability of the domain from a fixed an observer position. takes 4 REAL_T inputs: 2 coordinates of the observer and 2 coordinate of the location that is observed. The function outputs a single REAL_T: the observability. Defaults to inverse squared distance: vis = 1/(| x - y|^2 + eps ). Note that all observers have the same observability function but parametrized by different locations.
     * @param aSpeed function pointer REAL_T(REAL_T,REAL_T) to a speed function which defines the speed of the evader at all points in the domain. takes 2 REAL_T inputs: 2 coordinates of the evader. The function outputs a single REAL_T: the speed at that location. Defaults to constant aSpeed = 1.
     * @param aN integer defining the number of grid points in each ShortDirection.  Defaults to 501 (observed to be sufficient to resolve line of sight).
     * @param aPartialCostGrids boolean flag which determines whether the secondary linear equations "v_i" are computed and used to compute the subgradients.
     * @param aMinPhys real number, the lower bound defining the square domain [aMinPhys,aMaxPhys]x[aMinPhys, aMaxPhys]. Defaults to 0.
     * @param aMaxPhys real number, the upper bound defining the square domain [aMinPhys,aMaxPhys]x[aMinPhys, aMaxPhys].  Defaults to 1.
     */
    CStationaryMultipleEvaderPlanner( std::vector<std::vector<REAL_T>> aObsLocations,
                      std::vector<std::vector<REAL_T>> aEvadersSource,
                      std::vector<REAL_T> aTarget,
                      std::vector<REAL_T> aEvadersWeight,
                      CStationaryTerrain terrain = CStationaryTerrain(),
                      std::function<REAL_T(REAL_T, REAL_T,REAL_T, REAL_T)> aVis = &inverseSquared,
                      std::function<REAL_T(REAL_T, REAL_T)> aSpeed = &constant,
                      int aN = 501,
                      bool aPartialCostGrids = false,
                      REAL_T aMinPhys = 0,
                      REAL_T aMaxPhys = 1 );

    /**
     * The CAdversarialPlan Constructor.
     * This is a simplified constructor for a single observer. There is no weight input, and the source should be a vector of length 2.
     * @param aObsLocations a Nx2 std::vector of std::vector containing the locations of the observer, where N is the number of observer locations. For example, initialize to vector<REAL_T> aObsLocations = { { 0.1, 0.5}, {0.4, 0.6}} for two observers at locations (0.1,0.4) and (0.4,0.6).
     * @param aSource a length 2 std::vector containing the location of the source.
     * @param aTarget a length 2 std::vector containing the location of the target.
     * @param aTerrain a CStationaryTerrain object specifying the obstacles. A std::vector of std::vector of coordinates, defining rectangular obstacles can be passed too, see the CTerrain object constructor.
     * @param aVis function pointer REAL_T(REAL_T,REAL_T,REAL_T,REAL_T) to a observability function which defines the observability of the domain from a fixed an observer position. takes 4 REAL_T inputs: 2 coordinates of the observer and 2 coordinate of the location that is observed. The function outputs a single REAL_T: the observability. Defaults to inverse squared distance: vis = 1/(| x - y|^2 + eps ). Note that all observers have the same observability function but parametrized by different locations.
     * @param aSpeed function pointer REAL_T(REAL_T,REAL_T) to a speed function which defines the speed of the evader at all points in the domain. takes 2 REAL_T inputs: 2 coordinates of the evader. The function outputs a single REAL_T: the speed at that location. Defaults to constant aSpeed = 1.
     * @param aN integer defining the number of grid points in each ShortDirection.  Defaults to 501 (observed to be sufficient to resolve line of sight).
     * @param aPartialCostGrids boolean flag which determines whether the secondary linear equations "v_i" are computed and used to compute the subgradients.
     * @param aMinPhys real number, the lower bound defining the square domain [aMinPhys,aMaxPhys]x[aMinPhys, aMaxPhys]. Defaults to 0.
     * @param aMaxPhys real number, the upper bound defining the square domain [aMinPhys,aMaxPhys]x[aMinPhys, aMaxPhys].  Defaults to 1.
     */
    CStationaryMultipleEvaderPlanner(
        std::vector<std::vector<REAL_T>> aLocations,
        std::vector<REAL_T> aSource,
        std::vector<REAL_T> aTarget,
        CStationaryTerrain terrain = CStationaryTerrain(),
        std::function<REAL_T(REAL_T, REAL_T,REAL_T, REAL_T)> aVis = &inverseSquared,
        std::function<REAL_T(REAL_T, REAL_T)> aSpeed = &constant,
        int aN = 501 ,
        bool aPartialCostGrids = false,
        REAL_T aMinPhys = 0,
        REAL_T aMaxPhys = 1);

    /**
     * A function which computes the expected observability for a given probability distribution of observer locations, and for a specific point in the domain. Used as an input for the CFMM/CGradientDescentTracer objects.
     * @param aLambda Eigen::VectorXd of length equal to the possible of observer locations. A probability distribution over the observer positions.
     * @param aX real number, the physical x coordinate of a position in the domain.
     * @param aY real number, the physical y coordinate of a position in the domain.
     */
    virtual REAL_T getExpectedObservability( Eigen::VectorXd aLambda, REAL_T aX , REAL_T aY);

    /**
     * A helper function for the subgradient method.
     * For a given probability distribution over the observer locations, this function computes the optimal expected cost (the solution of the scalarized Eikonal at the source) which is the value of the objection function, and the vector of partial costs at the source which is the subgradient of the objective function. It uses the CFMM and CGradientDescentTracer objects to compute this. NOTE It returns the negative of the quantities expected, this is because the subgradient descent method works on a convex function, whereas this function is concave.
     * @param aLambda Eigen::VectorXd of length equal to the possible of observer locations. A probability distribution over the observer positions.
     * @param aValue output, a real number, the value of the objective function.
     * @param aSubgradient output, Eigen::VectorXd, the integrated observability with respect to individual observer locations.
     */
    virtual void getValueAndSubgradient(
      const Eigen::VectorXd aLambda,
      REAL_T& aValue,
      Eigen::VectorXd& aSubgradient ) override;


    /**
     * I/O function which writes the current state of all variable to file.
     * It Calls the I/O function of the CGridContext for all the single observer grids and for the Expected Obs grid, CGradientDescentTracer, and prints the position of source and target and observers.
     * @param aFilename string containing the name of the file for which to save all the data. Defaults to the empty string.
     */
    virtual void writeAllToFile(std::string aFilename = "");

    /**
     * writes current path to file.
     */
    virtual void saveStrategyToFile(std::string filename) override;

    virtual int getNumberOfObservers() override;

  private:
    int fNumObservers;  /**< Number of observer locations  */
    std::shared_ptr<std::vector<CStationaryObserver>> fObservers;  /**< pointer to vector of CStationaryObserver objects, which contains the information about each observer location, observability and line of sight */
    std::shared_ptr<std::vector<CGridContext>> fSingleObsGrid;  /**< pointer to vector of CGridContext which contains the properties of the associated linear PDEs for each observer location */
    std::shared_ptr<CGridContext> fExpectedObsGrid; /**< pointer to CGridContext which contains the properties of the scalarized Eikonal equation */
    std::shared_ptr<CFMM> fCFMMobject; /**< pointer to CFMMObject, respsonsible for the multiobjective Eikonal solve */
    std::vector<REAL_T> fTarget; /**< vector of length 2 containing the physical coordinates of the target */
    std::vector<std::vector<REAL_T>> fEvadersSource;  /**< vector of length 2 containing the physical coordinates of the sources */
    std::vector<REAL_T> fEvadersWeight;  /**< vector of length 2 containing the physical coordinates of the sources */
    std::vector<std::shared_ptr<CGradientDescentTracer>> fTracer; /**< pointed to fTracer object which does GradientDescent on the scalarized Eikonal (defined by the fExpectedObsGrid ) */


};

#endif
