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
* File: PaperExamples.hpp
*
* Author: Marc Aurèle Gilles
*
* Description: Definition of problems shown in figures in the manuscript.
*
* ==============================================================================
*/

#include "global_configuration.hpp"
#include "CStationaryMultipleEvaderPlanner.hpp"
#include "CStationaryMultipleEvaderMultipleTargetPlanner.hpp"
#include <vector>
#include <random>

using namespace memory;
using namespace std;

void TwoObsComputeAll( std::shared_ptr<CStationaryMultipleEvaderPlanner> StationaryPlan , string name ){
  // Compute Nash
  auto t1 = std::chrono::high_resolution_clock::now();
  std::shared_ptr<CFixedLambdaPlanner> FixedPlan(StationaryPlan);
  auto AdversarialPlan = make_unique<CAdversarialPlan>(FixedPlan);
  AdversarialPlan->findNash(name, 100);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "Nash computation took "
          << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count()
          << " seconds\n";

  StationaryPlan->writeAllToFile(name);

  // Compute whole PF with scalarization (to make part c. of Figs)
  //AdversarialPlan->GetParetoFront(name, 1000);

  REAL_T PreviousValue;
  Eigen::VectorXd Subgradient(2);
  Eigen::VectorXd CurrentLambda(2);

  CurrentLambda[0] =  0;
  CurrentLambda[1] =  1;
  StationaryPlan->getValueAndSubgradient( CurrentLambda, PreviousValue, Subgradient);
  StationaryPlan->saveStrategyToFile(name+"PathPure0");
  std::cout << "Pure plan costs 1:" << Subgradient << std::endl;
  CurrentLambda[0] =  1;
  CurrentLambda[1] =  0;
  StationaryPlan->getValueAndSubgradient( CurrentLambda, PreviousValue, Subgradient);
  StationaryPlan->saveStrategyToFile(name+"PathPure1");
  std::cout << "Pure plan costs 2:" << Subgradient << std::endl;


}


// Fig 3
void FigureThree(){

    //Define Observer locations, source, target.
    vector<vector<REAL_T> > ObserverLocations = { {0.95, 0.05},{0.4,0.6} };
    vector<REAL_T> source = {0.95, 0.93};
    vector<REAL_T> target = {0.02, 0.05};

    vector<vector<REAL_T>> Obstacles;

    //Define CAdversarialPlan object.
    auto StationaryPlan = make_shared<CStationaryMultipleEvaderPlanner>( ObserverLocations, source, target,Obstacles,inverseSquared, constant,501,true);
    std::shared_ptr<CFixedLambdaPlanner> FixedPlan(StationaryPlan);
    string name = "FigureThree";

    TwoObsComputeAll( StationaryPlan, name);

}


// Fig. 4
void FigureFour(){

  //Define Observer locations, source, target.
  vector<vector<REAL_T> > ObserverLocations = { {0.2, 0.8},{0.6,0.4} };
  vector<REAL_T> source = {0.95, 0.95};
  vector<REAL_T> target = {0.05, 0.05};

  vector<vector<REAL_T>> Obstacles;
  CStationaryTerrain terrain = CStationaryTerrain(Obstacles);

  //Define CAdversarialPlan object.
  auto StationaryPlan = make_shared<CStationaryMultipleEvaderPlanner>( ObserverLocations, source, target,terrain,inverseSquared, constant, 501, true);

  string name = "FigureFour";
  TwoObsComputeAll( StationaryPlan, name);

}


// Fig. 5
void FigureFive(){

    //Define Observer locations, source, target.
    vector<vector<REAL_T> > ObserverLocations = { {0.03, 0.97},{0.5,0.5} };
    vector<REAL_T> source = {0.95, 0.95};
    vector<REAL_T> target = {0.05, 0.05};

    vector<vector<REAL_T>> Obstacles;

    //Define CAdversarialPlan object.
    auto StationaryPlan = make_shared<CStationaryMultipleEvaderPlanner>( ObserverLocations, source, target,Obstacles,smallInverseSquared,constant, 501,true);
    string name = "FigureFive";
    TwoObsComputeAll( StationaryPlan, name);

}

// Fig. 6
void FigureSix(){
  string name = "FigureSix";
  //Define Observer locations, source, target.
  vector<vector<REAL_T> > ObserverLocations = { {0.8, 0.2},{0.15,0.9} };
  vector<REAL_T> source = {0.95, 0.93};
  vector<REAL_T> target = {0.02, 0.05};

  vector<vector<REAL_T>> Obstacles = { {0.35,0.7,0.4,0.6}};
  CStationaryTerrain terrain = CStationaryTerrain(Obstacles);

  //Define CAdversarialPlan object.
  auto StationaryPlan = make_shared<CStationaryMultipleEvaderPlanner>( ObserverLocations, source, target,terrain, inverseSquared, constant, 501);
  TwoObsComputeAll( StationaryPlan, name);


}

// Fig 9.
void FigureNine(){

    //Define Observer locations, source, target.
    vector<REAL_T> source = {0.5, 0.02};
    vector<REAL_T> target = {0.5, 0.98};
    string name = "FigureNine";
    vector<vector<REAL_T> > ObserverLocations = { {0.15, 0.5},{0.5,0.5},{0.85, 0.5} };

    vector<vector<REAL_T>> Obstacles = { {0.33,0.37,0.1,0.9},
                                         {0.63,0.67,0.1,0.9}};

    CStationaryTerrain terrain = CStationaryTerrain(Obstacles);

    auto t1 = std::chrono::high_resolution_clock::now();

    //Define CAdversarialPlan object.
    auto StationaryPlan = make_shared<CStationaryMultipleEvaderPlanner>( ObserverLocations, source, target,terrain );

    std::shared_ptr<CFixedLambdaPlanner> FixedPlan(StationaryPlan);

    auto AdversarialPlan = make_unique<CAdversarialPlan>(FixedPlan);
    AdversarialPlan->findNash(name, 300);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "f() took "
            << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count()
            << " seconds\n";
   StationaryPlan->writeAllToFile(name);

}

// Fig. 10
void FigureTen(){
    string name = "FigureTen";
    //Define Observer locations, source, target.
    vector<REAL_T> source = {0.6, 0.02};
    vector<REAL_T> target = {0.6, 0.7};

    vector<vector<REAL_T> > ObserverLocations = { {0.05, 0.95},
                                                //  {0.1, 0.25},
                                                  {0.85, 0.25},
                                                  {0.60, 0.45},
                                                  {0.96, 0.05},
                                                  {0.29, 0.55  },
                                                  {0.85,0.85}
                                                };

    vector<vector<REAL_T>> Obstacles = {
                                        {0.0,0.6,0.1,0.2}, //horizontal walls, from top bot, left->right
                                        {0.7,0.95,0.1,0.2},
                                        {0.5,0.8,0.3,0.4},
                                        {0.5,0.7,0.5,0.6},
                                        {0.1,0.5,0.7,0.8},
                                        {0.1,0.95,0.9,0.95},
                                        //vertical
                                        {0.1,0.2,0.3,0.8},
                                        {0.3,0.4,0.2,0.6},
                                        {0.45,0.5,0.5,0.7},
                                        {0.65,0.70,0.7,0.95},
                                        {0.75,0.8,0.3,0.8},
                                        {0.9,0.95,0.1,0.95},
                                        };

    CStationaryTerrain terrain = CStationaryTerrain(Obstacles);

    auto t1 = std::chrono::high_resolution_clock::now();

    //Define CAdversarialPlan object.
    auto StationaryPlan = make_shared<CStationaryMultipleEvaderPlanner>( ObserverLocations, source, target,terrain );

    std::shared_ptr<CFixedLambdaPlanner> FixedPlan(StationaryPlan);

    auto AdversarialPlan = make_unique<CAdversarialPlan>(FixedPlan);
    AdversarialPlan->findNash(name, 400);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "f() took "
            << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count()
            << " seconds\n";
   StationaryPlan->writeAllToFile(name);
}


// Fig. 11
void FigureEleven(){

  //Define Observer locations, source, target.
  vector<vector<REAL_T>> ObserverLocations = { {0.8, 0.2},{0.15,0.9} };

  //vector<vector<REAL_T>> source =  { {0.95, 0.4},{0.5, 0.1}  } ;
  vector<vector<REAL_T>> source =  { {0.95, 0.93},{0.2, 0.95}  };
  vector<vector<REAL_T>> target= {{0.02, 0.05},{0.1, 0.2  }};

  vector<REAL_T> weight = { 1,1};

  vector<vector<REAL_T>> Obstacles = { {0.35,0.7,0.4,0.6}};
  CStationaryTerrain terrain = CStationaryTerrain(Obstacles);

  auto t1 = std::chrono::high_resolution_clock::now();

  //Define CAdversarialPlan object.
  auto StationaryPlan = make_shared<CStationaryMultipleEvaderMultipleTargetPlanner>(ObserverLocations, source, target,weight,terrain , inverseSquared, constant, 501,true);

  std::shared_ptr<CFixedLambdaPlanner> FixedPlan(StationaryPlan);
    string name = "FigureEleven";
  auto AdversarialPlan = make_unique<CAdversarialPlan>(FixedPlan);
   AdversarialPlan->findNash(name, 600);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "f() took "
          << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count()
          << " seconds\n";
  StationaryPlan->writeAllToFile(name);

}


// Fig. 12
void FigureTwelve(){

    string name = "FigureTwelve";
    //Define Observer locations, source, target.
    vector<vector<REAL_T>> source = {{0.05,0.6},{0.6, 0.02}};
    vector<vector<REAL_T>> target = {{0.85,0.6},{0.6, 0.7}};
    vector<REAL_T> weight = {1,1};

    //Define Observer locations, source, target.
    vector<vector<REAL_T> > ObserverLocations = { {0.05, 0.95},
                                                //  {0.1, 0.25},
                                                  {0.85, 0.25},
                                                  {0.60, 0.45},
                                                  {0.96, 0.05},
                                                  {0.29, 0.55  },
                                                  {0.85,0.85}
                                                };


    vector<vector<REAL_T>> Obstacles = {
                                        {0.0,0.6,0.1,0.2}, //horizontal walls, from top bot, left->right
                                        {0.7,0.95,0.1,0.2},
                                        {0.5,0.8,0.3,0.4},
                                        {0.5,0.7,0.5,0.6},
                                        {0.1,0.5,0.7,0.8},
                                        {0.1,0.95,0.9,0.95},
                                        //vertical
                                        {0.1,0.2,0.3,0.8},
                                        {0.3,0.4,0.2,0.6},
                                        {0.45,0.5,0.5,0.7},
                                        {0.65,0.70,0.7,0.95},
                                        {0.75,0.8,0.3,0.8},
                                        {0.9,0.95,0.1,0.95},
                                        };

    CStationaryTerrain terrain = CStationaryTerrain(Obstacles);

    auto t1 = std::chrono::high_resolution_clock::now();

    //Define CAdversarialPlan object.
    auto StationaryPlan = make_shared<CStationaryMultipleEvaderMultipleTargetPlanner>(ObserverLocations, source, target,weight,terrain,inverseSquared, constant, 501, true );

    //Define CFixedLambdaPlanner object.
    std::shared_ptr<CFixedLambdaPlanner> FixedPlan(StationaryPlan);

    auto AdversarialPlan = make_unique<CAdversarialPlan>(FixedPlan);
    AdversarialPlan->findNash(name, 300);

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "f() took "
            << std::chrono::duration_cast<std::chrono::seconds>(t2-t1).count()
            << " seconds\n";
   StationaryPlan->writeAllToFile(name);

}
