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
* File: CGridContext.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: This class is used to handle  both the logical and the physical
* representation of a 2D regular grid of fixed spacing h in both the x and y
* directions. It depends on the boost::multi_array class
* (see memory_allocations.hpp), and it is the sole interface used by CFMM and
* CGradientDescentTracer. This class is an interface for between the underlying
* data structures and the rest of the code (CFMM and CGradientDescentTracer in
* particular). Only the grid values are stored as an array. The speed and cost
* functions are stored only as function pointers, which are treated as inputs.
*
* ==============================================================================
*/


#ifndef CGridContext_HPP
#define CGridContext_HPP
#include "boost/multi_array.hpp"
#include "global_configuration.hpp"
#include "memory_allocations.hpp"
#include "SimpleFunctions.hpp"
#include <math.h>
#include <string>

class CGridContext
{

private:

  // Properties of grid
  REAL_T fH; /**< Grid spacing: assuming to be the same in the two spatial variable */
  std::shared_ptr<memory::array2D_t<REAL_T>> fValues; /**< Grid of values. Stored as a pointer to a 2D boost::multi_array */
  REAL_T fMinX; /**< lower bound of x on the grid  */
  REAL_T fMinY; /**< lower bound of y on the grid  */
  REAL_T fMaxX; /**< upper bound of x on the grid  */
  REAL_T fMaxY; /**< upper bound of y on the grid  */
  std::function<REAL_T(REAL_T, REAL_T)> fSpeed; /**< Speed function, stored as a function pointer. This has to be given has an input.  */
  std::function<REAL_T(REAL_T, REAL_T)> fCost; /**< Speed function, stored as a function pointer. This is an input (or defaults to constant)  */

public:

  // setters
  void setSpeed(std::function<REAL_T(REAL_T, REAL_T)> aSpeed);
  void setCost(std::function<REAL_T(REAL_T, REAL_T)> aCost);
  void setValue(int aI, int aJ, REAL_T aValue );

  // getters

  // Grid-coordinate functions
  REAL_T getValue(int aI, int aJ ) const;
  REAL_T getSpeed(int aI, int aJ ) const;
  REAL_T getCost(int aI, int aJ ) const;

  // Physical-coordinate functions
  REAL_T getSpeedPhysical(REAL_T aX, REAL_T aY ) const;
  REAL_T getCostPhysical(REAL_T aX, REAL_T aY ) const;

  // Grid parameters
  int getGridSizeX() const;
  int getGridSizeY() const;
  REAL_T getH() const;
  REAL_T getMinX() const;
  REAL_T getMinY() const;
  REAL_T getMaxX() const;
  REAL_T getMaxY() const;

  // Mapping back and forth between grid and physical
  REAL_T xGridToPhysical(int aI) const;
  REAL_T yGridToPhysical(int aJ) const;
  int xPhysicalToGrid(REAL_T aX) const;
  int yPhysicalToGrid(REAL_T aY) const;

  // Constructors /d-tor
  void initCGridContext();
  CGridContext() = default;

  /**
   * The CGridContext Constructor.
   * This is the main constructor. It initializes the grid and computes the grid spacing
   * for a square grid with the same number of points.
   * @param aCost cost function pointer. If this isn't given then default constructor is called
   * @param aSpeed speed function pointer. Defaults to a constant speed.
   * @param aN an integer the number of grid points in each ShortDirection. Defaults to a constant speed. Defaults to 501.
   * @param aPhysMin a real number. The lower bound on the physical grid [aPhysMin ; aPhysMax] x [aPhysMin ; aPhysMax]
   * @param aPhysMax a real number. The upper bound on the physical grid [aPhysMin ; aPhysMax] x [aPhysMin ; aPhysMax]
   */
  CGridContext(  std::function<REAL_T(REAL_T, REAL_T)> aCost , \
   std::function<REAL_T(REAL_T, REAL_T)> aSpeed = &constant, int aN = 501, \
  REAL_T aPhysMin = 0, REAL_T aPhysMax = 1  );


  /**
   * A CGridContext Constructor.
   * A second constructor for convenience with order of arguments changed.
   */
  CGridContext( int aN , REAL_T aPhysMin = 0, REAL_T aPhysMax = 1, \
   std::function<REAL_T(REAL_T, REAL_T)> aCost = &constant , \
   std::function<REAL_T(REAL_T, REAL_T)> aSpeed = &constant );


  /**
   * A I/O member which prints the values, cost and speed grid to file.
   * @param aFilename a string which contains the name of the file to which the grids will be printed. The (cost/speed/value) grids will be printed to the file called "aFilename"+(Cost/Speed/Value)
   */
  void writeGridsToFile( std::string aFilename ) const;

};

// Inline functions must be in header file.
// All of these functions are called a lot.
inline void CGridContext::setValue(int aI, int aJ, REAL_T aValue ){
  (*fValues)[aI][aJ] = aValue;
}

inline REAL_T CGridContext::getValue(int aI, int aJ ) const
{
  return (*fValues)[aI][aJ];
}

inline REAL_T CGridContext::getSpeed(int aI, int aJ ) const
{
  return fSpeed(fMinX + aI*fH , fMinY + aJ*fH );
}

inline REAL_T CGridContext::getCost(int aI, int aJ ) const
{
    return fCost(fMinX + aI*fH , fMinY +aJ*fH );
}

inline REAL_T CGridContext::getSpeedPhysical(REAL_T aX, REAL_T aY ) const
{
  return fSpeed( aX , aY );
}

inline REAL_T CGridContext::getCostPhysical(REAL_T aX, REAL_T aY ) const
{
    return fCost( aX , aY );
}

inline REAL_T CGridContext::xGridToPhysical(int aI) const
{
    return fMinX + (REAL_T)aI * fH;
}

inline REAL_T CGridContext::yGridToPhysical(int aJ) const
{
    return fMinY + (REAL_T)aJ * fH;
}

inline int CGridContext::xPhysicalToGrid(REAL_T aX) const
{
    return round((aX + fMinX)/fH  );
}

inline int CGridContext::yPhysicalToGrid(REAL_T aY) const
{
    return round((aY + fMinY)/fH );
}

inline int CGridContext::getGridSizeX() const{
  return fValues->shape()[0];
}

inline int CGridContext::getGridSizeY() const{
  return fValues->shape()[1];
}
inline REAL_T CGridContext::getH() const{
  return fH;
}
inline REAL_T CGridContext::getMinX() const{
  return fMinX;
}
inline REAL_T CGridContext::getMinY() const{
  return fMinY;
}
inline REAL_T CGridContext::getMaxX() const{
  return fMaxX;
}
inline REAL_T CGridContext::getMaxY() const{
  return fMaxY;
}

#endif
