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
* File: OptimizationAlgorithm.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See OptimizationAlgorithm.hpp
*
* ==============================================================================
*/

#include"OptimizationAlgorithm.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>    // std::max
#include "misc.hpp"
using namespace Eigen;
using namespace std;



Eigen::VectorXd ProjectedSubgradientMethod(
   std::function<void(Eigen::VectorXd, REAL_T&, Eigen::VectorXd& )> aValueSubgradientFun , \
   std::function<void(Eigen::VectorXd&)> aProjectionFun, \
   Eigen::VectorXd aX0, \
   int aMaxIter,\
   bool aSqrtStepSize ,\
   string aFilename,
   double aTolerance
 ){

  int dimension = aX0.size();
  REAL_T StepSizeInit = 1; //initial step size.
  int StepsBeforeStagnation = aMaxIter; //initial step size.
  REAL_T StepSize = StepSizeInit;
  REAL_T CurrentValue, TempValue, BestValue, StagValue;
  int Stagnation, MaxStagnation;
  bool Done;
  VectorXd CurrentSubgradient = VectorXd::Zero(dimension);
  VectorXd NewSubgradient(dimension);
  VectorXd TempX(dimension);
  VectorXd CurrentX(dimension);
  VectorXd BestX(dimension);

  //
  Stagnation = 0;
  MaxStagnation = 30; // Maximum number of steps without improvement before iteration stops.
  Done = false; // flag that indicates end of iterations.

  vector<double> allvalues; // Used to store the values at iteration.
  vector<double> allsubgradients; // Used to store the subgradients at iteration.
  vector<double> alliterates; // Used to store the subgradients at iteration.

  // Initialize iterates
  CurrentX = aX0;
  // Project initial guess.
  aProjectionFun(CurrentX);
  cout << "Start X: " << CurrentX << endl;
  BestX = CurrentX;
  aValueSubgradientFun( CurrentX, CurrentValue, CurrentSubgradient);

  // Initial values = best values
  BestValue = CurrentValue;
  StagValue = CurrentValue;

  // Set scale initial step size;
  StepSizeInit = StepSizeInit/CurrentSubgradient.norm();
  StepSize = StepSizeInit;

  for ( int iter = 0 ; iter < aMaxIter ; ++iter){


    // The subgradient step
    TempX = CurrentX - StepSize*CurrentSubgradient;

    // Projection
    aProjectionFun(TempX);

    // Get new subgradient and value.
    aValueSubgradientFun( TempX, TempValue, NewSubgradient);

    if (aSqrtStepSize){
      StepSize = (REAL_T)StepSizeInit*1.0/((sqrt(iter) + 1));
    } else {
      StepSize = (REAL_T)StepSizeInit*1.0/(((iter) + 1));
    }

    CurrentSubgradient = NewSubgradient;
    CurrentValue = TempValue;
    CurrentX = TempX;

    // check for stagnation. Don't start before StepsBeforeStagnation iterations (until we have seen enough decrease in the stepsize)
    if ( (CurrentValue + aTolerance > StagValue) && (iter > StepsBeforeStagnation) ){
      Stagnation++;
    } else {
      Stagnation = 0;
      StagValue = CurrentValue; // value at last not stagnated
    }

    if (CurrentValue <= BestValue){
      BestValue = CurrentValue;
      BestX = CurrentX;
    }

    // if stagnated for too long, end iteration
    if (Stagnation >= MaxStagnation){
      cout <<"Iteration has stagnated for " << Stagnation << ", with tolerance of  ";
      cout << aTolerance << ". Ending iterations." << endl;
      break;
    }

    // --------------------------------------------------------------------//
    // Save to File and Print.
    // --------------------------------------------------------------------//
    if (aFilename.compare("") != 0){
      allvalues.push_back(CurrentValue);
      for ( int i = 0 ; i < CurrentSubgradient.size(); ++i){
      allsubgradients.push_back(CurrentSubgradient[i]);
      alliterates.push_back(CurrentX[i]);
      }
    }
    cout << "iteration: " << iter << "| value: " << CurrentValue;
    cout << "| x:  ";
    for ( int k = 0 ; k < dimension ; ++k){
      printf("%.3e, ", CurrentX[k] );
    }
    cout << "| subgradient:  ";
    for ( int k = 0 ; k < dimension ; ++k){
      printf("%.3e, ", CurrentSubgradient[k] );
    }
    cout<< "Stagnated Steps: " << Stagnation << "| stepsize:"<< StepSize << endl;

  }

  if (aFilename.compare("") != 0){
    io::write_vector_to_file(aFilename+"IteratesValues", allvalues);
    io::write_vector_to_file(aFilename+"IteratesSubgradients",allsubgradients);
    io::write_vector_to_file(aFilename+"Iterates",alliterates);
  }
  return BestX;
}

Eigen::VectorXd StochasticProjectedSubgradientMethod(
   std::vector<std::function<void(Eigen::VectorXd, REAL_T&, Eigen::VectorXd& )>> aValuesSubgradientsFun , \
   std::function<void(Eigen::VectorXd&)> aProjectionFun, \
   Eigen::VectorXd aX0, \
   int aMaxIter,\
   int aBatchSize,\
   string aFilename
 ){

  int dimension = aX0.size();
  REAL_T StepSizeInit = 1; //initial step size. Actually this*norm subgradient
  REAL_T StepSize, TempValue;
  VectorXd Subgradient = VectorXd::Zero(dimension);
  VectorXd SummedSubgradient = VectorXd::Zero(dimension);

  VectorXd CurrentX(dimension);

  vector<double> allsubgradients; // Used to store the subgradients at iteration.
  vector<double> alliterates; // Used to store the subgradients at iteration.

  // Initialize iterates
  CurrentX = aX0;
  // Project initial guess.
  aProjectionFun(CurrentX);
  cout << "Start X: " << CurrentX << endl;

  //define vector of order:
  std::random_device rd;// only used once to initialise (seed) engine
  std::mt19937 rng(rd());// random-number engine used
  std::uniform_int_distribution<int> uni(0,aValuesSubgradientsFun.size() - 1); // guaranteed unbiased

  for ( int iter = 0 ; iter < aMaxIter ; ++iter){
    SummedSubgradient = VectorXd::Zero(dimension);

    for (int i = 0 ; i < aBatchSize ; ++i){
      auto random_int = uni(rng); // pick random planner to do a partial subgradient step.
      //int random_int = (iter*aBatchSize + i) %20; // This does cyclic instead.
      aValuesSubgradientsFun[random_int]( CurrentX, TempValue, Subgradient);
      SummedSubgradient += Subgradient;
    }

    // This is one of the many stepsizing rule which gives O(1/sqrt(k)) convergence for subgradient method for convex fcn.
    StepSize = (REAL_T)StepSizeInit*1.0/(sqrt((iter*aBatchSize) + 1));

    // The subgradient step
    CurrentX = CurrentX - StepSize*SummedSubgradient;

    // Projection
    aProjectionFun(CurrentX);

    // --------------------------------------------------------------------//
    // Save to File
    // --------------------------------------------------------------------//
    if (aFilename.compare("") != 0){
      for ( int i = 0 ; i < Subgradient.size(); ++i){
      allsubgradients.push_back(Subgradient[i]);
      alliterates.push_back(CurrentX[i]);
      }
    }

    cout << "iteration: " << iter;
    cout << "| x:  ";
    for ( int k = 0 ; k < dimension ; ++k){
      printf("%.3e, ", CurrentX[k] );
    }
    cout << "| subgradient:  ";
    for ( int k = 0 ; k < dimension ; ++k){
      printf("%.3e, ", Subgradient[k] );
    }
    cout << "| stepsize:"<< StepSize << endl;
  }

  if (aFilename.compare("") != 0){
    io::write_vector_to_file(aFilename+"IteratesSubgradients",allsubgradients);
    io::write_vector_to_file(aFilename+"Iterates",alliterates);
  }

  // TODO: return an average? Asymptotically log(k) better.
  return CurrentX;
}
