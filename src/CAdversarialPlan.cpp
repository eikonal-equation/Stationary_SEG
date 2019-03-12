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
* File: CAdversarialPlan.cpp
*
* Author: Marc Aurèle Gilles
*
* Description: See CAdversarialPlan.hpp
*
* ==============================================================================
*/

#include "CAdversarialPlan.hpp"
#include "SimpleFunctions.hpp"
#include "SimplexProjection.hpp"
#include <iostream>
#include "misc.hpp"
#include <math.h>
#include <functional>
#include <Eigen>
#include "OptimizationAlgorithm.hpp"
#include "QuadProg++.hh"

using namespace Eigen;
using namespace memory;
using namespace std;
using namespace std::placeholders;

CAdversarialPlan::CAdversarialPlan( std::shared_ptr<CFixedLambdaPlanner> aFixedLambdaPlan){
  fFixedLambdaPlanner = aFixedLambdaPlan;
}

void CAdversarialPlan::findNash(string aFilename, int aMaxIter){

  // Define Objective function & subgradient Projected Subgradient Descent
  std::function<void(VectorXd, REAL_T&, VectorXd& )> ValAndSubg =  \
               std::bind( &CFixedLambdaPlanner::getValueAndSubgradient, fFixedLambdaPlanner, _1, _2, _3 );

  // Define Projection for Projected Subgradient Descent
  std::function<void(VectorXd& )> Projection = SimplexProjection;

  // Initial guess is uniform probability
  VectorXd aX0 =  VectorXd::Constant(fFixedLambdaPlanner->getNumberOfObservers(), (REAL_T) 1/fFixedLambdaPlanner->getNumberOfObservers() );

  // Find best strategy for observer.
  VectorXd Lambda = ProjectedSubgradientMethod( ValAndSubg ,  Projection, aX0, aMaxIter,\
     false, aFilename );

  // Find best strategy for evader.
  findMultiplePaths(Lambda, aFilename);

}

void CAdversarialPlan::findNashStochastic(string aFilename, int aMaxIter, int aBatchSize){

  // Define Objective function & subgradient Projected Subgradient Descent
  vector<function<void(VectorXd, REAL_T&, VectorXd& )>> ValAndSubg(fFixedLambdaPlanner->getNumberOfPlans());
  for( int i = 0 ; i < fFixedLambdaPlanner->getNumberOfPlans(); ++i){
    ValAndSubg[i] = std::bind( &CFixedLambdaPlanner::getValuesAndSubgradients, fFixedLambdaPlanner, _1, _2, _3, i );
  }

  // Define Projection for Projected Subgradient Descent
  std::function<void(VectorXd& )> Projection = SimplexProjection;

  // Initial guess is uniform probability
  VectorXd aX0 =  VectorXd::Constant(fFixedLambdaPlanner->getNumberOfObservers(), (REAL_T) 1/fFixedLambdaPlanner->getNumberOfObservers() );

  // Find best strategy for observer.
  VectorXd Lambda = StochasticProjectedSubgradientMethod( ValAndSubg ,  Projection, aX0, aMaxIter, aBatchSize,aFilename );

  // Find best strategy for evader.
  findMultiplePaths(Lambda, aFilename);

}


void CAdversarialPlan::findMultiplePaths(VectorXd aLambda, string aFilename){
  int k, NonZeros, i , j;
  int fNumObservers = aLambda.size();
  VectorXd Subgradient(fNumObservers);
  VectorXd Lambda(fNumObservers);
  VectorXd TempLambda(fNumObservers);
  VectorXd ReducedLambda;
  VectorXd Direction; // Direction of perturbation
  MatrixXd A;  // Matrix of Partial costs: Used to perform the least square.
  VectorXd rhs; // Right hand side: constant entry equal to Value.
  VectorXd theta;  // Solution of least square.
  std::vector<bool> keep(fNumObservers);
  REAL_T Value, stepsize, ZeroThresh, ResidualTresh, AlmostZeroObservability;


  // Parameters. Fixed by experience. TODO: Make stepsize adaptive.
  stepsize = 1e-4; // the size of the perturbation
  ZeroThresh = 5e-3; // Probabilities below this threshold are set to 0.
  ResidualTresh = 1e-3; // How close the characterization based on the residual is to 0.
  AlmostZeroObservability = 1e-6; // parameter which gives a little observability to observer positions which should have none at all (because their probability was set to 0). This is essentially for tiebreaking. Can be set to zero to turn this option off.

  NonZeros = 0; //number of observer location which have non-zero probability.
  Lambda = aLambda;


  // Set small probabilities to zero.
  for ( k = 0 ; k < fNumObservers; ++ k){
    if ( Lambda[k] < ZeroThresh){
      Lambda[k] = 0;
      keep[k] = false;
    } else {
      keep[k] =  true;
      NonZeros += 1;
    }
  }

  ReducedLambda.resize(NonZeros);
  Direction.resize(NonZeros);

  // Project the vector with zero-ed component onto probability simplex.
  GetShortVector( ReducedLambda, Lambda, keep);
  SimplexProjection( ReducedLambda);
  GetLongVector( ReducedLambda, Lambda, keep);

  cout << "Optimal lambda: " << aLambda << "Lambda used: " << Lambda << endl;
  // Get New Value and subgradient from treshholded probability.
  // Recall that this function really returns the negative of the value and the subgradient but in this situation it does not matter.
  fFixedLambdaPlanner->getValueAndSubgradient( Lambda, Value, Subgradient);
  cout << " Total cost of best path found: " << Value << endl;

  int MaxNumberPaths = 2*NonZeros + 1;
  // Matrix of Partial costs: Used to perform the least square.
  A= MatrixXd::Zero(NonZeros,MaxNumberPaths);
  // Right hand side: constant entry equal to Value.
  rhs = MatrixXd::Constant(NonZeros, 1, Lambda.transpose()*Subgradient);


  // Store of probabilities and costs.
  array2D_t<REAL_T> Costs = memory::allocate_2d_array<REAL_T>( MaxNumberPaths , fNumObservers + 1);
  array2D_t<REAL_T> Probabilities = memory::allocate_2d_array<REAL_T>( MaxNumberPaths, fNumObservers );

  k = 0 ;
  //  Partial Costs;
  for (int i = 0; i < fNumObservers; ++i){
      Costs[k][i] = Subgradient[i];
  }
  cout << "Partial Costs of best path found: " << Subgradient << endl;

  // In last entry of Costs matrix, store expected cost.s
  Costs[k][fNumObservers] = Value;

  //  Store Probability distribution
  for( int i = 0 ; i < fNumObservers; ++ i) {
      Probabilities[k][i] = Lambda[i];
  }

  // Store first strategy.
  fFixedLambdaPlanner->saveStrategyToFile(aFilename + "path" + std::to_string(k));


  for ( k = 1; k < MaxNumberPaths  ; ++k){

      // Extract appropriate component of subgradients to form A matrix
      i = 0 ; j = 0 ;
      while(  j < NonZeros ){
          if (keep[i]== true){
            A(j,k - 1) =  Subgradient[i];
            ++j;
          }
          ++i;
      }

      // Solve Least square problem
      //theta.resize(k);
      theta = CallQuadProg(A.block(0,0,NonZeros,k),rhs);

      // The perturbation direction is the one in the direction of the subproblem.
      Direction = A.block(0,0,NonZeros,k)*theta - rhs;
      cout << "Norm of residual at step " << k << " is " << Direction.norm()/(fabs(Value)*NonZeros) << endl;
      // if Residual is small, then we are done.
      if (Direction.norm()/(fabs(Value)*NonZeros)  < ResidualTresh){
        cout << "Small residual! Terminating!" << endl;
        k++;
        break;
      }

      if ( theta[k-1]  < 1e-8){

        cout << "Last path found not used. Alg failed. Terminating. " << endl;
        break;
      }

      cout << " A mat : " << endl  << A.block(0,0,NonZeros,k) << endl;
      cout << "rhs : " << endl  << rhs << endl;
      cout << "theta: " << endl << theta << endl;
      cout << "Direction: " << endl << Direction << endl;
      cout << "Lambda: " << endl << Lambda << endl;


      TempLambda = Lambda;

      stepsize = 1e-4; // smallest size of perturbation.
      bool newPathFound = false;
      while ( (newPathFound == false)  && (stepsize < 1) ){
        // Perturb and project back onto probabilty simplex
        GetShortVector( ReducedLambda, TempLambda, keep);
        ReducedLambda = ReducedLambda - Direction*stepsize;
        SimplexProjection(ReducedLambda);
        GetLongVector( ReducedLambda, TempLambda, keep);

        // Add tiny values for zeros
        for ( int i = 0 ; i < Subgradient.size(); ++i){
          if (TempLambda[i] == 0){
            TempLambda[i] = AlmostZeroObservability;
          }
        }


        // Compute new subgradient & value
        fFixedLambdaPlanner->getValueAndSubgradient( TempLambda, Value, Subgradient);
        cout << "Subgradient found: " << Subgradient << endl;
        //check if similar to previous subgradients
        int NDifferentPath = 0 ;

        for ( int j = 0; j < k ; ++j ){
            double diff = 0;
            for ( int i = 0; i < Subgradient.size() ; ++i ){
              // compute L^2 distance in the vector of individual costs.
              diff += (Subgradient[i] - Costs[j][i])*(Subgradient[i] - Costs[j][i]);
            }
            if ( sqrt(diff) > 1e-2*Subgradient.norm() ){
              // if the difference is greater than thresh, we have found a new path.
              NDifferentPath++;
            }
        }
        // If the current subgradient is different from all others, then we have a new path.
        if (NDifferentPath == k){
          newPathFound = true;
        }
        else{ // otherwise increase stepsize.
          cout << "Increasing perturbation size at step " << k << endl;
          stepsize = stepsize*2;
        }
      }

      cout << " value: " << Value << endl;

      //Store Partial Costs
      for (int i = 0; i < fNumObservers; ++i){
          Costs[k][i] = Subgradient[i];
      }
      //Store Expected cost
      Costs[k][fNumObservers] = Value;

      // Store Probabilities
      for( int i = 0 ; i < fNumObservers; ++ i) {
          Probabilities[k][i] = TempLambda[i];
      }

      // Save current strategy.
      fFixedLambdaPlanner->saveStrategyToFile(aFilename + "path" + std::to_string(k));

  }

  // save to file.
  io::write_to_file<REAL_T>(aFilename + "Costs", Costs, MaxNumberPaths,  fNumObservers + 1);
  io::write_to_file<REAL_T>(aFilename + "Probabilities", Probabilities, MaxNumberPaths, fNumObservers );

  // recompute theta
  theta = CallQuadProg(A.block(0,0,NonZeros,k-1),rhs);

  cout << "Theta : " << theta << endl;
  std::vector<REAL_T> thetaVec(theta.data(), theta.data() + theta.size());
  io::write_vector_to_file(aFilename + "Theta",  thetaVec);

  // Recompute once more with input aLambda so that writing to file uses the best possible lambda
  fFixedLambdaPlanner->getValueAndSubgradient( aLambda, Value, Subgradient);

}

void CAdversarialPlan::GetShortVector(VectorXd& aShort, VectorXd aLong,  std::vector<bool> aKeep ){
  int j = 0 ;
  for ( int k = 0 ; k < aLong.size() ; ++k){
    if (aKeep[k] == true) {
      aShort[j] = aLong[k];
      ++j;
    }
  }
}


void CAdversarialPlan::GetLongVector(VectorXd aShort, VectorXd& aLong,  std::vector<bool> aKeep ){
  int j = 0 ;
  for ( int k = 0 ; k < aLong.size() ; ++k){
    if (aKeep[k] == true) {
      aLong[k] = aShort[j];
      ++j;
    }
  }
}

// ONLY FOR 2 OBSERVER CASE.
void CAdversarialPlan::GetParetoFront(
   std::string aFilename ,
   int aMaxIter
 ){
   int NPlans = fFixedLambdaPlanner->getNumberOfPlans();
   vector<REAL_T> PF;
   vector<vector<REAL_T>> PFSingle;
   if (NPlans > 1){
     PFSingle.resize(NPlans);
    }
   VectorXd TempX(2);
   VectorXd Subgradient(2);
   REAL_T TempValue;
   VectorXd SumSubgradient(2);

   for ( int i = 0 ; i < aMaxIter + 1; ++i){
     TempX[0] = float(i)/aMaxIter;
     TempX[1] = 1 - float(i)/aMaxIter;
     if ( NPlans > 1) {
       SumSubgradient <<  0, 0;
       for (int j = 0 ; j < NPlans; ++j ){
         fFixedLambdaPlanner->getValuesAndSubgradients( TempX, TempValue, Subgradient,j);
         PFSingle[j].push_back(Subgradient[0]);
         PFSingle[j].push_back(Subgradient[1]);
         SumSubgradient[0] += Subgradient[0];
         SumSubgradient[1] += Subgradient[1];
       }
       PF.push_back(SumSubgradient[0 ]);
       PF.push_back(SumSubgradient[1 ]);
     } else {
       fFixedLambdaPlanner->getValueAndSubgradient( TempX, TempValue, Subgradient);
       PF.push_back(Subgradient[0 ]);
       PF.push_back(Subgradient[1 ]);
     }
   }
  io::write_vector_to_file(aFilename + "PF",  PF);

  if ( NPlans > 1) {
    for (int j = 0 ; j < NPlans; ++j ){
      io::write_vector_to_file(aFilename + "PFSingleEvader"+to_string(j) ,  PFSingle[j]);
    }
  }

}

VectorXd CAdversarialPlan::CallQuadProg(
  MatrixXd aA,  // Matrix of Partial costs: Used to perform the least square.
  VectorXd aRhs // Right hand side: constant entry equal to Value.
 ){

   /* Sets up the QP.
   The problem is in the form:

   min 0.5 * x G x + g0 x
   s.t.
       CE^T x + ce0 = 0
       CI^T x + ci0 >= 0
   */
   int dim = aA.cols();
   MatrixXd G = aA.transpose()*aA;
   VectorXd g0 =  -1*aA.transpose()*aRhs;
   MatrixXd CE =  MatrixXd::Constant(dim,1,1);
   VectorXd ce0 = VectorXd::Constant(1,-1);
   MatrixXd CI(dim,2*dim);
   CI.block(0,0,dim,dim) = MatrixXd::Identity(dim,dim);
   CI.block(0,dim,dim,dim) = -1*MatrixXd::Identity(dim,dim);
   VectorXd ci0(2*dim);
   ci0.head(dim) = VectorXd::Constant(dim,0);
   ci0.tail(dim) = VectorXd::Constant(dim,1);
   VectorXd theta(dim);
   double d = QuadProgPP::solve_quadprog(G,g0,CE, ce0, CI,ci0,theta);

   return theta;
}
