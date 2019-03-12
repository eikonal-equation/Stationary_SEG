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
* File: CAdversarialPlan.hpp
*
* Author: Marc Aurèle Gilles.
*
* Description: Computes the Nash equilibrium of a surveillance-evasion game.
* First the probability distribution is computed using a projected subgradient
* descent method, where the subgradient is the vector of partial costs
* (integrated observability from each position) at the source and the value of
* the function is the scalarized cost (expected integrated observability) at the
* source. Then the optimal probability distribution is perturbed to obtain a set
* of different paths which form a Nash equilibrium.
* It uses the class CFixedLambdaPlanner.
*
* ==============================================================================
*/
#ifndef CADVERSARIALPLAN_HPP
#define CADVERSARIALPLAN_HPP

#include "math.h"

// STL
#include <vector>
#include <string>

// Other file/classes
#include "global_configuration.hpp"
#include "CFixedLambdaPlanner.hpp"
#include "CStationaryTerrain.hpp"

#include <Eigen>

class CAdversarialPlan
{


  public:
    CAdversarialPlan() = default;

    /**
     * The CAdversarialPlan Constructor.
     */
    CAdversarialPlan( std::shared_ptr<CFixedLambdaPlanner> );



    /**
     * This is the main function of the class which seeks the Nash Equilibrium for the problem.
     * It can be called after the object was initialized. It calls the subgradient method to find then best probability distribution of observers, then calls the method findMultiplePaths to find the best strategy for the evader.
     * @param aFilename string containing the name of the file for which to save all the data. Defaults to the empty string.
     * @param aMaxIter integer, the maximum number of subgradient method iteration. Defaults to 300.
     */
    void findNash(std::string aFilename = "", int maxiter = 300 );

    /**
     * This is a stochastic version of the findNash. It calls the stochastic subgradient method to find then best probability distribution of observers, then calls the method findMultiplePaths to find the best strategy for the evader.
     * @param aFilename string containing the name of the file for which to save all the data. Defaults to the empty string.
     * @param aMaxIter integer, the maximum number of subgradient method iteration. Defaults to 300.
     * @param aBatchSize integer, the number of observer position used for each iteration of the stochastic subgradient
     */
    void findNashStochastic(std::string aFilename = "", int maxiter = 300, int aBatchSize = 1 );



    /**
     * This is the function which finds the optimal strategy for the observer.
     * This function perturbs the given probability distribution aLambda, (which should be the optimal strategy for the observer) in order to find multiple path associated with near-aLambda distributions.
     * @param aLambda vector of doubles, probability distribution over observer positions.
     * @param aFilename string containing the name of the file for which to save all the data. Defaults to the empty string.
     */
    void findMultiplePaths(Eigen::VectorXd aLambda, std::string aFilename = "");

    /** Computes the convex part of a pareto front using the scalarization approach. Only works for 2 observers.
    * @param aFilename string containing the name of the file for which to save all the data. Defaults to the empty string.
    * @param aMaxiter integer, the number of grid points of [0,1] used.
    */
    void GetParetoFront(
      std::string aFilename = "", \
      int aMaxiter = 100
    );


  private:
    /** The CFixedLambdaPlanner object that is used    */
    std::shared_ptr<CFixedLambdaPlanner> fFixedLambdaPlanner;

    /**
     * A helper function for the findMultiplePaths function.
     * Given a long Eigen::VectorXd, and a std::vector of boolean indicating which index of the long vector should be kept, it returns the corresponding shorted Eigen::VectorXd aShort.
     * @param aShort output, Eigen:vector of length M containing the entries in aLong which have corresponding "1" entry in aKeep, where M = sum(aKeep).
     * @param aLong input, Eigen:vector of length N.
     * @param aKeep input, std::vector<bool> of length N with M true/1 entries.
     */
    static void GetShortVector(Eigen::VectorXd& aShort, Eigen::VectorXd aLong,  std::vector<bool> aKeep );

    /**
     * Inverse operation of GetShortVector.
     */
    static void GetLongVector(Eigen::VectorXd aShort, Eigen::VectorXd& aLong,  std::vector<bool> aKeep );

    /**
     * A helper function which calls the Library QuadProg++ to solve the quadratic program that shows in the perturbation step.
     * @param aA input, Matrix of Partial costs: Used to perform the least square.
     * @param aRhs input, Right hand side: constant entry equal to Value.
     */
    Eigen::VectorXd CallQuadProg(
      Eigen::MatrixXd aA,
      Eigen::VectorXd aRhs
    );


};

#endif
