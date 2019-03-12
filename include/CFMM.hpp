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
* File: CFMM.hpp
*
* Author: Marc Aurèle Gilles (based on Zachary Clawson's implementation of
* VanillaFMM)
*
* Description: This is a multiobjective Fast Marching Method class. It works on
* a 2D regular grid and a 5 point stencil. It assumes that the spacing in both
* the x and y direction is the same but the number of grid points can differ.
* The target set must be a single point. It solves the multiobjective problem
* by scalarization, which involves solving linear PDEs associated with the
* solution of the Eikonal. The solution to the Eikonal is stored in the fPrimary
*  variable, and the solution of the associated linear PDEs is stored in the
* fSecondary variable.
* The FMM implementation uses the boost::heap class, and CGridContext class.
*
* ==============================================================================
*/


#ifndef CFMM_HPP
#define CFMM_HPP
#include "CGridContext.hpp"
#include <boost/heap/binomial_heap.hpp>
#include "global_configuration.hpp"

class CFMM
{
  public:

    //c-tor
    CFMM() = default;
    CFMM(std::shared_ptr<CGridContext> aPrimary);

    /**
     * The CFMM Constructor
     * This is the main CAdversarialPlan object constructor.
     * @param aPrimary pointer to CGridContext object containing the primary grid on which the scalarized Eikonal equation is solved.
     * @param aSecondary pointer to vector of CGridContext objects containing the grids defining the associated linear PDEs.
     */
    CFMM(std::shared_ptr<CGridContext> aPrimary, std::shared_ptr<std::vector<CGridContext>> aSecondary);


    /**
     * The fast marching method.
     * This function actually starts the marching. The solution of the Eikonal is stored in the fValues array of fPrimary (accessible by getValue/setValue), and the solution of the associated linear PDEs are stores in the fValues arrays of fSecondary.
     * @param aTargetX real number which defines the physical x coordinate of the target position.
     * @param aTargetY real number which defines the physical y coordinate of the target position.
     */
    void march(REAL_T aTargetX, REAL_T aTargetY);

  private:

    // A GripPoint Class ot be used by the heap.
    class CHeapGP{
      public:
        int fI;
        int fJ;
        REAL_T fValue;
        CHeapGP( int aI, int aJ, REAL_T aValue): fI(aI), fJ(aJ), fValue(aValue) {};
    };
    // Struct comparaison which is required by boost::heap
    struct compare_CHeapGP
    {
        bool operator()(const CHeapGP& aPoint1, const CHeapGP& aPoint2) const
        {
            return aPoint1.fValue > aPoint2.fValue;
        }
    };

    // Typedef Heap types to make it easier to read.
    typedef boost::heap::binomial_heap<CHeapGP, boost::heap::compare<compare_CHeapGP> > CFMMHeap_t;
    typedef typename boost::heap::binomial_heap<CHeapGP, boost::heap::compare<compare_CHeapGP> >::handle_type handle_t;


    std::shared_ptr<CGridContext> fPrimary;  /**< A pointer to a GridContext object which contains all of the information (cost, speed, physical grid) about the Eikonal PDE */
    std::shared_ptr<std::vector<CGridContext>> fSecondary; /**< A pointer to a vector of GridContext objects which contains all of the information (cost, speed, physical grid) about the linear PDEs associated which define the "secondary" cost */
    int fNumSecondary;  /**< Number of secondary variable */

    std::unique_ptr<memory::array2D_t<STATUS_T>> fStatus; /**< The grid of status of each grip point. */
    std::unique_ptr<memory::array2D_t<handle_t>> fHeapPointers; /**< The grid of backpointers for the heap */


    /**
     * FMM Initialization.
     * This function sets the status of all nodes and adds the Target to the heap.
     * @param CFMMHeap_t& aCFMMHeap Boost::heap passed by value which gets initialized.
     * @param aTargetX int x logical coordinate of grid point of the target.
     * @param aTargetY int y logical coordinate of grid point of the target.
     */
    void initializeCFMM( CFMMHeap_t& aCFMMHeap, int aTargetX, int aTargetY );

    /** A helper function which updates neighbors of a gridpoint and add them to the heap.
    * @param aCFMMHeap boost heap object
    * @param aCurrent_i int x logical coordinate of grid point
    * @param aCurrent_j int y logical coordinate of grid point
    */
    void updateNeighbors( CFMMHeap_t& CFMMheap, const  int aCurrent_i, const  int aCurrent_j);

    /**
     * A helper function to figure out if grid point (aI,aJ) is in the domain.
     * @param aI int x logical coordinate
     * @param aJ int J logical coordinate
     */
    bool inDomain( const int aI, const int aJ ) const;

    /** A helper function which returns the value (and values of the associated linear PDEs) of the smaller of the two horizontal neighbors.
    * @param aI int x logical coordinate
    * @param aJ int J logical coordinate
    * @param aVal a vector containg in aVal[0] the value of the smaller horizontal neighbor, and in aVal[1:fNumSecondary] the associated secondary values.
    */
    void smallerHneighbor(const int aI, const int aJ, std::vector<REAL_T>& aVal) const;

    /** A helper function which returns the value (and values of the associated linear PDEs) of the smaller of the two vertical neighbors.
    * @param aI int x logical coordinate
    * @param aJ int J logical coordinate
    * @param aVal a (fNumSecondary+1) length vector containg in aVal[0] the value of the smaller vertical neighbor, and in aVal[1:fNumSecondary] the associated secondary values.
    */
    void smallerVneighbor(const int aI, const int aJ, std::vector<REAL_T>& aVal) const;

    /** A compute function which computes the Eikonal update and the update of the associated linear PDEs.
    * @param aF a (fNumSecondary+1) length vector of effective speed (= speed / cost) of the fPrimary grid (in aF[0] )and secondary grids (in aF[1:fNumSecondary]).
    * @param aVal1 a (fNumSecondary+1) length vector containg in the values of the smaller vertical neighbor.
    * @param aVal2 a (fNumSecondary+1) length vector containg in the values of the smaller horizontal neighbor.
    */
    void computeFromTwoNeighbors(const std::vector<REAL_T>& aF, const std::vector<REAL_T>& aVal1, const std::vector<REAL_T>& aVal2, std::vector<REAL_T>& aNewVal);


    /** Update a gridpoint. Compute the update, assign, and return if updated.
    * @param aI int x logical coordinate
    * @param aJ int J logical coordinate
    */
    bool updateGP(const int aI, const int aJ);



};

#endif
