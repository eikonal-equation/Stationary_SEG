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
* File: CGradientDescentTracer.hpp
*
* Author: Marc Aurèle Gilles (based on Zachary Clawson's implementation of
* VanillaFMM)
*
* Description: This is the Gradient Tracer class, which is responsible for
* finding the optimal path given the solution of the Eikonal equation. It
* assumes that a 2D regular grid with the same spacing in both directions. It
* uses bilinear interpolation of the solution of the Eikonal on the grid to
* define a continuous solution. The target and source are assumed to be single
* points.This implementation does the gradient descent with a crude grid search
* and refines with Golden Section Search.
* It depends on the CGridContext class and is called by CAdversarialPlan.
*
* ==============================================================================
*/

#ifndef GRADIENTDESCENTRACER_HPP
#define GRADIENTDESCENTRACER_HPP

//-- STL -----------------------------------------------------------------------------------------//
#include <vector>
#include <string>

//-- Global --------------------------------------------------------------------------------------//
#include "global_configuration.hpp"
#include "CGridContext.hpp"

//-- typedefs --------------------------------------------------------------------------------//
struct Point2d {
  REAL_T x, y;
  Point2d(REAL_T a, REAL_T b) {
    x = a;
    y = b;
  }
} ;

//-- Data Structure ------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
typedef std::vector<Point2d> Path;


class CGradientDescentTracer {
public:


    //-- c-tor/d-tor -----------------------------------------------------------------------------//
    CGradientDescentTracer() = default;

    /**
     * The CGradientDescentTracer Constructor
     * This is the main constructor.
     * @param aContext the CGridContext object which contains the information about the PDE.
     * @param aSourcex a real number defining the physical x coordinate of the source.
     * @param aSourcey a real number defining the physical y coordinate of the source.
     * @param aTargetx a real number defining the physical x coordinate of the target.
     * @param aTargety a real number defining the physical y coordinate of the target.
     */
    CGradientDescentTracer(std::shared_ptr<CGridContext> aContext, REAL_T aSourcex, REAL_T aSourcey, REAL_T aTargetx, REAL_T aTargety );

    //-- main ------------------------------------------------------------------------------------//
    /**
    * A path printing function.
    * Computes the optimal path and prints it to file specified by aFilename
    * @param aFilename a string containing the name of the file.
    */
    void   printOptimalPathToFile( std::string aFilename = "optpath");

    /**
    * The main optimal path function.
    */
    Path   getOptimalPath();

    REAL_T fSourcex, fSourcey, fTargetx, fTargety; /** Source and Target X and Y coordinates */

    std::shared_ptr<CGridContext> fPrimary;  /**  The Gridcontext which defines the Eikonal PDE. See CGridContext.hpp */

private:

    //-- helpers ---------------------------------------------------------------------------------//
    REAL_T distanceToSource(const REAL_T x, const REAL_T y) const;
    REAL_T distanceToTarget(const REAL_T x, const REAL_T y) const;

    /**
    * Bilinear interpolation on the value grid.
    * This function computes the bilinear interpolation of the solution of the Eikonal equation.
    * @param aX a real number containing the physical x coordinate.
    * @param aY a real number containing the physical y coordinate.
    * @return the interpolated value of CGridContext.fValues at (x,y).
    */
    REAL_T getValue(const REAL_T aX, const REAL_T aY);

    /**
    * Performs Golden Section Search to find the optimal direction of motion.
    * @param aX a real number containing the physical x coordinate of the current point.
    * @param aY a real number containing the physical y coordinate of the current point.
    * @param aL a real number, lower bound on angle to search.
    * @param aH a real number, upperbound on angle to search.
    * @param aStep a real number, the stepsize of the gradient descent.
    * @param aStep aTol a real number, tolerance on the angle.
    */
    REAL_T GoldenSectionSearch(REAL_T aX, REAL_T aY, REAL_T aL, REAL_T aH, REAL_T aStep, REAL_T aTol);

    /**
    * Helper function Computes the value function off the grid by bilinear interpolation.
    * @param aCurrX a real number containing the physical x coordinate of the current point.
    * @param aCurrY a real number containing the physical y coordinate of the current point.
    * @param aAngle a real number, angle of the direction step.
    * @param aStep a real number, stepsize of the direction step.
    */
    REAL_T TracerHelper(REAL_T aCurrX, REAL_T aCurrY, REAL_T aAngle, REAL_T aStep);
};

//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

#endif /* CGradientDescentTracer_hpp */
