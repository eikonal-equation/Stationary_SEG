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
* File: OptimizationAlgorithm.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: Optimization Algorithms to be used as the "outer" loop.
* Uses the Eigen library.
*
* ==============================================================================
*/


#ifndef OPTIMIZATIONALGORITHM_HPP
#define OPTIMIZATIONALGORITHM_HPP

#include "math.h"
// STL
#include <vector>
#include <string>
#include <Eigen>
#include "global_configuration.hpp"


/**
 * A projected subgradient descent method.
 * It attemps to solve the optimization problem : min_{x \in S } f(x)
 *  It implements the iteration:
 *
 * for k=0:maxIter
 *  x_{k+1} = Pi(x_k - s* \partial f x_{k} )
 *  end
 *
 * where Pi is an orthogonal projection onto the set S and \partial f x is a subgradient of f. s is by default 1/(norm(partial aX0)*(k+1)
 * This implementation is however specialized to our specific problem, and it is reflected in a couple of ways:
 * Convergence is computed by a stagnation coefficient, by only after a fixed number of iterations has been attempted. The logic behind this is that we have a step rule so that for the first few iterations, we may be taking steps which are way too large.
 * @param aValueSubgradientFun a function pointer which returns the value and subgradient. The function shoud take three arguments: 1st argument is argument is the input x , second argument is the output f(x) and the third argument is the output \partial f (x).
 * @param aProjectionFun a function pointer which performs the orthogonal projection onto the set S in place. The function should have one input argument which is also used for output.
 * @param aX0 a Eigen::VectorXd containing the initial guess of the method.
 * @param aMaxIter integer, the maximum number of iterations
 * @param aSqrtStepSize. If on, uses stepsize of 1/sqrt(k) instead of 1/k.
 * @param aFilename string, prints the iterates to this field.
 * @param aTolerance double, tolerance if none of the last 10 iterates have made improvement of at least aTolerance, the iteration ends.
*/
Eigen::VectorXd ProjectedSubgradientMethod(
   std::function<void(Eigen::VectorXd, REAL_T&, Eigen::VectorXd& )> aValueSubgradientFun , \
   std::function<void(Eigen::VectorXd&)> aProjectionFun, \
   Eigen::VectorXd aX0, \
   int aMaxiter = 100,\
   bool aSqrtStepSize = false,\
   std::string aFilename = "",
   double aTolerance = 1e-6
 );


 /**
  * A stochastic projected subgradient descent method.
  * It attemps to solve the optimization problem : min_{x \in S } f(x)
  *  It implements the iteration:
  *
  * for k=0:maxIter
  *  x_{k+1} = Pi(x_k - s* g x_{k} )
  *  end
  *
  * where Pi is an orthogonal projection onto the set S and g is a stochastic estimate of the subgradient, i.e. E[g] = \partial f. s is by def 1/(sqrt(k)+1)
  * @param aValuesSubgradientsFun a std::vector of function pointer which returns the value and subgradient with respect to one of the plans. Each function shoud take three arguments: 1st argument is argument is the input x , second argument is the output f(x) and the third argument is the output \partial f (x).
  * @param aProjectionFun a function pointer which performs the orthogonal projection onto the set S in place. The function should have one input argument which is also used for output.
  * @param aX0 a Eigen::VectorXd containing the initial guess of the method.
  * @param aMaxIter integer, the maximum number of iterations
  * @param aBatchSize int, Size of batch that is used to define the stochastic estimate of the subgradient. Defaults to 1.
  * @param aFilename string, prints the iterates to this field.
 */
 Eigen::VectorXd StochasticProjectedSubgradientMethod(
    std::vector<std::function<void(Eigen::VectorXd, REAL_T&, Eigen::VectorXd& )>> aValuesSubgradientsFun , \
     std::function<void(Eigen::VectorXd&)> aProjectionFun, \
     Eigen::VectorXd aX0, \
     int aMaxiter = 100,\
     int aBatchSize = 1,\
     std::string aFilename = ""
);


#endif
