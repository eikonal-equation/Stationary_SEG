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
* File: CFixedLambdaPlanner.hpp
*
* Author: Marc Aurèle Gilles.
*
* Description: This is the abstract evader planner class, meant as an interface
* for CAdversarialPlan.hpp. The main function that children of this class should
* implement is getValueAndSubgradient, which is called in the
* subgradient/perturbation methods of CAdversarialPlan.
*
* ==============================================================================
*/

#ifndef CFIXEDLAMBDAPLANNER_HPP
#define CFIXEDLAMBDAPLANNER_HPP
#include <Eigen>
#include "global_configuration.hpp"
class CFixedLambdaPlanner{
  public:
    /**
     * The destructor
     */
    virtual ~CFixedLambdaPlanner() {};

   /* Function which computes value and subgradient of objective.
    * This is the main function to be implemented by children, it should have the following form:
    * @param aLambda (input) a VectorXd of Eigen, the probability of each observer location.
    * @param aValue (output) a real number, the value of the objective function at lambda
    * @param aSubgradient (output) a VectorXd of Eigen, the subgradient of the objective function at lambda.
    */
    virtual void getValueAndSubgradient(  const Eigen::VectorXd aLambda,
      REAL_T& aValue,
      Eigen::VectorXd& aSubgradient) = 0;

    /* Function which computes value and subgradient of objective with respect to a single
     * observer position. To be used by the stochastic gradient descent.
     * It should have the following form:
     * @param aLambda (input) a VectorXd of Eigen, the probability of each observer location.
     * @param aValue (output) a real number, the value of the objective function at lambda
     * @param aSubgradient (output) a VectorXd of Eigen, the subgradient of the objective function at lambda.
     * @param i (input) int, individual observer position to be used.
     */
     virtual void getValuesAndSubgradients(  const Eigen::VectorXd aLambda,
       REAL_T& aValue,
       Eigen::VectorXd& aSubgradient,
       int i );


    /* Helper function which returns the number of possible observer locations.
     */
    virtual int getNumberOfObservers() = 0;

    /* Helper function which returns the number of possible observer locations.
     */
    virtual int getNumberOfPlans();

    /* This saves the current strategy to file. Useful since this object is continually reused in CAdversarialPlan.
     */
    virtual void saveStrategyToFile(std::string filename) = 0;

};

#endif
