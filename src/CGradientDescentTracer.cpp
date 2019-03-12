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
* File: CGradientDescentTracer.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CGradientDescentTracer.hpp
*
* ==============================================================================
*/

//-- Base ----------------------------------------------------------------------------------------//
#include "CGradientDescentTracer.hpp"
#include <math.h>       /* fabs */
//-- STL -----------------------------------------------------------------------------------------//
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

// -- Constructor
CGradientDescentTracer::CGradientDescentTracer( shared_ptr<CGridContext> aContext, REAL_T aSourcex, REAL_T aSourcey, REAL_T aTargetx, REAL_T aTargety ){
  fPrimary = aContext;
  fSourcex = aSourcex;
  fSourcey = aSourcey;
  fTargetx = aTargetx;
  fTargety = aTargety;
}


inline REAL_T CGradientDescentTracer::distanceToSource(const REAL_T ax, const REAL_T ay) const {
    return sqrt( (fTargetx - ax)*(fTargetx - ax) +  (fTargety - ay)*(fTargety - ay));
}


inline REAL_T CGradientDescentTracer::distanceToTarget(const REAL_T ax, const REAL_T ay) const {
    return sqrt( (fTargetx - ax)*(fTargetx - ax) +  (fTargety - ay)*(fTargety - ay));

}


REAL_T CGradientDescentTracer::getValue(const REAL_T axP, const REAL_T ayP) {
    if (axP >= fPrimary->getMinX() && axP <= fPrimary->getMaxX() && ayP >= fPrimary->getMinY() && ayP <= fPrimary->getMaxY()) {
        int xL, yL, m, n;
        REAL_T hx, hy;

        hy = fPrimary->getH();
        hx = fPrimary->getH();
        xL = floor( (axP - fPrimary->getMinX()) / fPrimary->getH() );
        yL = floor( (ayP - fPrimary->getMinY()) /fPrimary->getH()  );
        m = fPrimary->getGridSizeX();
        n = fPrimary->getGridSizeY();

        REAL_T dx, dy, xi, eta;

        if (xL == axP * (m - 1) && yL == ayP * (n - 1)) // Check if point is on the corner
            return fPrimary->getValue(xL,yL);
        else if (xL == m -1) { // Check if point is a side of the square
            dy  = ayP - fPrimary->yGridToPhysical(yL);
            eta = (hy - dy) / hy;
            return eta * fPrimary->getValue(xL,yL) + (1 - eta) * fPrimary->getValue(xL,yL+1);
        }
        else if (yL == n-1) {
            dx  = axP - fPrimary->xGridToPhysical(xL);
            xi  = (hx - dx) / hx;
            return xi * fPrimary->getValue(xL,yL) + (1 - xi) * fPrimary->getValue(xL+1,yL);
        }
        else if (   fPrimary->getValue(xL,yL)== INF && fPrimary->getValue(xL+1,yL) == INF
                 && fPrimary->getValue(xL,yL+1) == INF && fPrimary->getValue(xL+1,yL+1) == INF)
            return INF;
        else {  // If not on the boundary, perform bilinear interpolation.

            dx  = axP - fPrimary->xGridToPhysical(xL);
            dy  = ayP - fPrimary->yGridToPhysical(yL);
            xi  = (hx - dx) / hx;
            eta = (hy - dy) / hy;

            REAL_T left_contrib     =   eta*fPrimary->getValue(xL,yL)      +   (1-eta)*fPrimary->getValue(xL,yL+1);
            REAL_T right_contrib    =   eta*fPrimary->getValue(xL+1,yL)    +   (1-eta)*fPrimary->getValue(xL+1,yL+1);
            return xi * left_contrib    +   (1 - xi) * right_contrib;

        }
    } else {
        return INF;
    }
}



Path CGradientDescentTracer::getOptimalPath() {
    Path path;

    path.push_back( Point2d( fSourcex ,fSourcey ) );
    REAL_T distToTar    = distanceToTarget(fSourcex, fSourcey);   // (curr_x,curr_y) distance to target
    REAL_T tau          = (fPrimary->getH())/ 10;  // time step

    // how close to target for termination
    REAL_T distThresh   = 2*tau;
    REAL_T GSCTol = 1e-5; // Golden Section search tolerance

    REAL_T best_theta, best_grid_theta, best_GSC_theta;

    int nFromGSC = 0, nFromGrid = 0;

    // LL is the number of directions tos earch in.
    // Should be divisible by 4 so that the 4 grid direction are searched.
    const int LL = 40;

    while (distToTar > distThresh) {
        REAL_T min_grid_val = INF;
        REAL_T curr_x = path[path.size() - 1].x;
        REAL_T curr_y = path[path.size() - 1].y;
        REAL_T best_x = -1;
        REAL_T best_y = -1;
        best_grid_theta = -1;
        const REAL_T speed = fPrimary->getSpeed(curr_x, curr_y);
        // performs rough grid search over directions
        for (int i = 0; i < LL; ++ i) {
            const REAL_T theta       = 2 * PI * (i) / LL;
            const REAL_T new_x       = curr_x + cos(theta) * tau * speed;
            const REAL_T new_y       = curr_y + sin(theta) * tau * speed;
            const REAL_T temp_val    = getValue(new_x, new_y); // this is
            if(temp_val < min_grid_val) {
                best_grid_theta = theta;
                min_grid_val = temp_val;
            }
        }

        // perform Golden Section Search over the small subinterval
        REAL_T best_GSC_theta = GoldenSectionSearch(curr_x, curr_y, best_theta - 2*PI/LL , best_theta + 2*PI/LL, tau, GSCTol);
        REAL_T min_val = TracerHelper(curr_x,curr_y,best_GSC_theta,tau);

        // pick out best between grid search and GSC.
        bool fromgrid = true;
        if ( min_val < min_grid_val){
          best_theta = best_GSC_theta;
          nFromGSC++;
          fromgrid = false;
        } else{
          best_theta = best_grid_theta;
          nFromGrid++;
        }

        best_x = curr_x + cos(best_theta) * tau * speed;
        best_y = curr_y + sin(best_theta) * tau * speed;
        min_val = getValue(best_x, best_y);

        assert(!isinf(min_val));

        // If we have not moved, search failed. Increase step sizes.

        // Checks to make sure Gradient Tracer is working properly.
        if( (fabs(best_x - path[path.size() - 1].x) +  fabs(best_y - path[path.size() - 1].y)) < tau/10) { cout << "Error 1" << fromgrid<<endl;}

        if ( (fabs(path[path.size() - 2].x - best_x) +  fabs(path[path.size() - 2].y - best_y)) < tau/10 ) { cout << "Error 2" << fromgrid <<endl;}

        if ( min_val == INF) { cout << "Error 3" << fromgrid<<endl;}

        if( (fabs(best_x - path[path.size() - 1].x) +  fabs(best_y - path[path.size() - 1].y)) < tau/10 \
        || (fabs(path[path.size() - 2].x - best_x) +  fabs(path[path.size() - 2].y - best_y)) < tau/10 \
        || min_val == INF) {
            tau = 2 * tau;
            distThresh   = 2*distThresh;
            cout << "Pathing failed at ";
            cout << "Coord: (" << best_x << ","<< best_y <<"). Increasing step size" << endl;
            path.pop_back();
            continue;
        }

        distToTar   = distanceToTarget(best_x, best_y);

        // Add new point to path
        path.push_back( Point2d(best_x, best_y) );

    }

    path.push_back( Point2d( fTargetx ,fTargety ) );
    return path;
}

void CGradientDescentTracer::printOptimalPathToFile( string aFilename){
    std::ofstream out( "output/" + aFilename , std::ios::binary );
    Path path = getOptimalPath();
    for (int i = 0; i < path.size(); ++ i) {
      out.write ((char*) &path[i].x, sizeof(REAL_T) );
      out.write ((char*) &path[i].y, sizeof(REAL_T) );
    }
}

REAL_T CGradientDescentTracer::GoldenSectionSearch(REAL_T aX, REAL_T aY, REAL_T aL, REAL_T aH, REAL_T aStep, REAL_T aTol) {
    // (x,y) is current position
    //
    // aL is lower bound, aH is upper bound
    //
    REAL_T bestFound = aL;
    REAL_T a = aL;
    REAL_T b = aH;
    REAL_T gr = (sqrt(5) +1 )/2; // golden ratio
    REAL_T c = b - (b - a) / gr;
    REAL_T d = a + (b - a) / gr;

    while (abs(c - d) > aTol) {
        if (TracerHelper(aX,aY,c,aStep) <= TracerHelper(aX,aY,d,aStep)) {
          // truncate left part of interval
            b = d;
            bestFound = c;
        }else {
          // truncate right part of interval
            a = c;
            bestFound = d ;
        }
        // we recompute both c and d here to avoid loss of precision which may lead to incorrect results or infinite loop
        c = b - (b - a) / gr;
        d = a + (b - a) / gr;
    }

    return bestFound;

}


REAL_T CGradientDescentTracer::TracerHelper(REAL_T aCurrX, REAL_T aCurrY, REAL_T aAngle, REAL_T aStep){
  const REAL_T new_x = aCurrX + cos(aAngle) * aStep * fPrimary->getSpeed(aCurrX, aCurrY);
  const REAL_T new_y = aCurrY + sin(aAngle) * aStep * fPrimary->getSpeed(aCurrX, aCurrY);
  return getValue(new_x, new_y);
}


//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
