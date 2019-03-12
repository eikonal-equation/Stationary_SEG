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
* File: CStationaryObserver.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: The stationary Observer class. It is a child class of CObserver.
* It owns an abstract CTerrain which defines the visibility. The role of this
* class is to find out the observability at each point in the domain.
*
* ==============================================================================
*/

#ifndef CSTATIONARYOBSERVER_HPP
#define CSTATIONARYOBSERVER_HPP
#include <math.h>
#include "global_configuration.hpp"
#include "memory_allocations.hpp"
#include "CGridContext.hpp"
#include "CStationaryObstacle.hpp"
#include "CStationaryTerrain.hpp"
#include <limits>
#include "SimpleFunctions.hpp"
#include "CObserver.hpp"

class CStationaryObserver : public CObserver
{
  public:

    /**
     * The CStationaryObserver constructor
     * This is the main constructor. It initializes the properties and computes the shadow zones.
     * @param aPosX real number, the x physical location of the observer
     * @param aPosY real number, the y physical location of the observer
     * @param aTerrain CStationaryTerrain object containing the information about the obstacles.
     * @param aVis a function pointer to a observability function. Defaults to inverse Squared.
     * @param aN size of grid on which the shadow zones are computed.
     * @param aPhysMin lower bound on square domain on which shadows is computed. The domain is [aPhysMin, aPhysMax]x[aPhysMin,aPhysMax]. Defaults to 0.
     * @param aPhysMax upper bound on square domain on which shadows is computed. The domain is [aPhysMin, aPhysMax]x[aPhysMin,aPhysMax]. Defaults to 1.
     */
    CStationaryObserver( REAL_T aPosX, REAL_T aPosY, \
      std::shared_ptr<CStationaryTerrain> aTerrain =  std::make_shared<CStationaryTerrain>() ,\
      std::function<REAL_T(REAL_T, REAL_T,REAL_T,REAL_T)> aObs = &inverseSquared,\
      int aN = 501, \
      REAL_T aPhysMin = 0, \
      REAL_T aPhysMax = 1);
    CStationaryObserver() = default;


    /**
     * Returns the obstructed observability
     */

    virtual REAL_T getObservability( REAL_T aX, REAL_T aY, REAL_T aT = 0) const override;
    // getters
    REAL_T getPosX();
    REAL_T getPosY();

  private:
    REAL_T fPosX, fPosY; /**< X and Y physical coordinate of observer */
    std::function<REAL_T(REAL_T, REAL_T,REAL_T,REAL_T)> fUnobstructedObservability;  /**< The observability function */
    std::shared_ptr<CGridContext> fShadows;  /**< GridContext which contains the shadow zones (line of sight information) */


    /**
     * This function computes the shadow zones from the observer location, using the information about the obstacles.
     * It uses two FMM calls to do this. The first one computes the minimum distance between the observer position and all other points in the domain when there is no obstacles, then second one computes the same distance where there are obstacles. If the distances are significantly different for a given point, then this point must be out of sight for the observer.
     * @param aN size of grid on which the shadow zones are computed.
     * @param aPhysMin lower bound on square domain on which shadows is computed. The domain is [aPhysMin, aPhysMax]x[aPhysMin,aPhysMax]. Defaults to 0.
     * @param aPhysMax upper bound on square domain on which shadows is computed. The domain is [aPhysMin, aPhysMax]x[aPhysMin,aPhysMax]. Defaults to 1.
     */
    virtual void computeShadows( int aN, REAL_T aPhysMin, REAL_T aPhysMax );

    static const double ObservabilityInShadows; // small constant defining observability in shadow.


};

inline REAL_T CStationaryObserver::getObservability( REAL_T aX, REAL_T aY, REAL_T aT ) const {
         if (fTerrain->isNotInObstacles(aX, aY) ){
              return fShadows->getValue( fShadows->xPhysicalToGrid( aX),fShadows->xPhysicalToGrid( aY))  \
                      * fUnobstructedObservability(fPosX, fPosY, aX, aY) + ObservabilityInShadows;
          } else {
          return std::numeric_limits<REAL_T>::infinity();
        }
}

inline REAL_T CStationaryObserver::getPosX(){
  return fPosX;
}
inline REAL_T CStationaryObserver::getPosY(){
  return fPosY;
}

#endif
