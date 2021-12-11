#include <ilcplex/ilocplex.h>
ILOSTLBEGIN
#include <cstdio>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <stdlib.h>
#include <float.h>
#include <fstream>
#include <string>
#include <sstream>
#include <cstring>
#include <vector>
#include <cstdlib>
#include <time.h>

#include <memory>
#include <cassert>
#include <functional>

#include <lemon/list_graph.h>
#include <lemon/concepts/graph_components.h>
#include <lemon/connectivity.h>
#include <lemon/concepts/graph.h>
#include <lemon/kruskal.h>
#include <lemon/min_cost_arborescence.h>
#include <lemon/adaptors.h>
#include <lemon/concepts/maps.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include "instance.h"
#include "formulation.h"
#include "graph.h"
#include "formulation.h"

#define INF DBL_MAX
#define EPS 1e-4 // epsilon useed for violation of cuts

using namespace std;
typedef IloArray<IloNumArray>    IloNumMatrix;
typedef IloArray<IloIntArray>    IloIntMatrix;
typedef IloArray<IloIntMatrix>    IloIntMatrix3D;
typedef IloArray<IloNumVarArray> IloNumVarMatrix;
typedef IloArray<IloNumVarMatrix> IloNumVarMatrix3D;
typedef ListGraph::NodeMap<int> IdNodeMap;
typedef ListGraph::EdgeMap<int> IdEdgeMap;

double bestIncumbent = INF;
IloNum timeBestSolutionFound = 0.0;
IloNum timeStart = 0.0;
IloNum lastTimeRecorded = 0.0;
IloNum solvingRoot = 0.0;
IloNum lbRoot = 0.0;
IloNum ubRoot = 0.0;
IloNum gapOnRoot = 0.0;
IloNum cutAdded = 0.0;
IloNum cutBicliqueAdded = 0.0;
IloNum cutKTreeAdded = 0.0;
IloNum cutGenKTreeAdded = 0.0;
IloNum cutTiming = 0.0;
IloInt countCallback = 0;
IloInt nextCallback = 1;
std::vector<IloNum> LOWER_BOUND;
std::vector<IloNum> UPPER_BOUND;
std::vector<IloNum> PROG;


/******************************************************************/
/*                   		 INFOS		      	                  */
/******************************************************************/
ILOMIPINFOCALLBACK1(infoCallback,
					IloCplex, cplex)
{
	IloNum time = cplex.getCplexTime() - timeStart;
	if ( hasIncumbent() ) {
		if(getIncumbentObjValue() <= bestIncumbent - 0.1){
			bestIncumbent = getIncumbentObjValue();
			timeBestSolutionFound = time;
		}
	}
	if(getNnodes() == 1){
		// cout << endl << "node 1" << endl;
		lastTimeRecorded = time;
		PROG.push_back(lastTimeRecorded);
		IloNum lb = getBestObjValue();
		lbRoot = lb;
		LOWER_BOUND.push_back(lb);
		if ( hasIncumbent() ) {
			IloNum ub = getIncumbentObjValue();
			UPPER_BOUND.push_back(ub);
			ubRoot = ub;
		}
		else{
			UPPER_BOUND.push_back(0.0);
			ubRoot = INF;
		}
		solvingRoot = time;
		gapOnRoot = getMIPRelativeGap();
	}
	if((getNnodes() >= 2) && (time >= lastTimeRecorded + 5.00)){
		
		// cout << endl << "record" << endl;
		lastTimeRecorded = time;
		PROG.push_back(lastTimeRecorded);
		IloNum lb = getBestObjValue();
		LOWER_BOUND.push_back(lb);
		if ( hasIncumbent() ) {
			IloNum ub = getIncumbentObjValue();
			UPPER_BOUND.push_back(ub);
		}
		else{
			UPPER_BOUND.push_back(0.0);
		}
	}
}



class Fixed{
public:
	vector<bool> F_ZERO;
	vector<bool> F_ONE;
	
	Fixed();
	Fixed(const vector<bool> & tocopy0, const vector<bool> & tocopy1): F_ZERO(tocopy0), F_ONE(tocopy1) {}
	Fixed(const Fixed & tocopy): F_ZERO(tocopy.getF_ZERO()), F_ONE(tocopy.getF_ONE()) {}
	
	vector<bool> getF_ZERO() const{return F_ZERO;}	
	vector<bool> getF_ONE() const{return F_ONE;}	
	void copy(const Fixed & tocopy) {
		F_ZERO = tocopy.getF_ZERO();
		F_ONE = tocopy.getF_ONE();
	}
};

class GraphMaps{
public:
	std::vector< std::shared_ptr<ListGraph> > g;
	std::vector< std::shared_ptr<IdNodeMap> > IdNode;
	std::vector< std::shared_ptr<IdEdgeMap> > IdEdge;
	
	GraphMaps(){}
};

Fixed zeroSetting(Fixed & subproblem, const Instance &instance, vector<int> &a){
	// cout << "Entering zeroSetting" << endl;

	std::fill(a.begin(), a.end(), -1);

	const int CLIENTS = instance.getNbClients();
	const int VEHICLES = instance.getNbVehicles();
	// const int STATIONS = instance.getNbStops();
	const int STATIONS = instance.getNbPointsOfMaxInt();

	
	Fixed subproblem_prime(subproblem);
	/*
	for (int j = 1; j < VEHICLES; ++j) {
		if(subproblem_prime.F_ZERO[j] == 0){
			subproblem_prime.F_ZERO[j] = 1;
		}
	}
	*/
	vector<int> max_vehicle(CLIENTS, -1);
	
	// int station = v;
	// cout << "station " << station << endl;
	int firstClient = 0;
	max_vehicle[firstClient] = 0;
	for (int i = 1; i < CLIENTS; ++i){
		int client = i;
		int previousClient = i-1;
		// cout << "client " << client << endl;
		//Zero setting
		if (max_vehicle[previousClient] == VEHICLES - 1) {
			max_vehicle[client] = max_vehicle[previousClient];
		}
		else{
			//int previousMax = max_vehicle_previous(max_vehicle, instance, client);
			if (subproblem.F_ZERO[client*VEHICLES + (max_vehicle[previousClient] + 1)] == 1){
				max_vehicle[client] = max_vehicle[previousClient];
			}
			else {
				max_vehicle[client] = max_vehicle[previousClient] + 1;
			}
		}
	}
	
	for (int i = 0; i < CLIENTS; ++i){
		a[i] = max_vehicle[i];
		// cout << "a[" << client << "] = " << a[client] << endl;
		for (int j = a[i]+1; j < VEHICLES; ++j) {
			if(subproblem_prime.F_ZERO[i*VEHICLES + j] == 0){
				subproblem_prime.F_ZERO[i*VEHICLES + j] = 1;
			}
		}
	}
	// cout << "return zeroFixing" << endl;
	return subproblem_prime;
}
Fixed oneSetting(Fixed & subproblem, const Instance &instance, vector<int> &a){
	Fixed subproblem_prime(subproblem);
	int CLIENTS = instance.getNbClients();
	int VEHICLES = instance.getNbVehicles();
	for (int i = 1; i < CLIENTS; ++i) {
		if(subproblem_prime.F_ONE[i*VEHICLES + a[i]] == 0){
			Fixed subproblem_temp(subproblem_prime);
			subproblem_temp.F_ZERO[i*VEHICLES + a[i]] = 1;
			Fixed subproblem_star(zeroSetting(subproblem_temp, instance, a) );
			bool contradiction = false;
			for (unsigned int it = 0; it < subproblem_star.F_ZERO.size(); ++it){
				if ( (subproblem_star.F_ZERO[it] == 1) && (subproblem_star.F_ONE[it] == 1) ){
					contradiction = true;
				}
			}
			if (contradiction){
				
				subproblem_prime.F_ONE[i*VEHICLES + a[i]] = 1;
				for (int j = 0; j < VEHICLES; ++j){
					if (j != a[i]){
						subproblem_prime.F_ZERO[i*VEHICLES + j] = 1;
					}
				}
			}
		}
	}
	return subproblem_prime;
}

ILOBRANCHCALLBACK2(orbitopalFixing, const Instance &, instance, IloNumVarMatrix &, X) {
	
	// cout << "begin branch" << endl;
	if(getIncumbentObjValue() - getObjValue() <= 0.999){
		// cout << "Prune node." << endl;
		prune();
		return;
	}
	else{
		int CLIENTS = instance.getNbClients();
		int VEHICLES = instance.getNbVehicles();
		vector<bool> F_ZERO(CLIENTS*VEHICLES, 0);
		vector<bool> F_ONE(CLIENTS*VEHICLES, 0);
		vector<int> a;
		a.resize(CLIENTS);
		
		//Initialization of F_0 and F_1
		for (int i = 0; i < CLIENTS; ++i) {
			for (int j = 0; j < instance.getNbVehicles(); ++j) {
				if (getUB(X[i][j]) <= 0.0001) {
					F_ZERO[i*VEHICLES + j] = 1;
				}
				if (getLB(X[i][j]) >= 0.9999 ) {
					F_ONE[i*VEHICLES + j] = 1;
				}
			}
		}
		// Zero and One Settings
		Fixed subproblem(F_ZERO, F_ONE);
		// cout << "zeroFixing" << endl;
		// subproblem.copy( zeroSetting(subproblem, instance, a) );
		Fixed subproblem_2 = zeroSetting(subproblem, instance, a);
		// cout << "oneFixing" << endl;
		// Fixed subproblem_prime = subproblem_2;
		Fixed subproblem_prime = oneSetting( subproblem_2, instance, a);	
		// Make branches
		int nbBranches = getNbranches();
		for (int b = 0; b < nbBranches; ++b) {
			IloNumVarArray vars(getEnv());
			IloNumArray bounds(getEnv());
			IloCplex::BranchDirectionArray dirs(getEnv());
			getBranch(vars, bounds, dirs, b);
			// cout << "Branching variable: ";
			// for (int v = 0; v < vars.getSize(); ++v){
				// cout << vars[v] << " to "<< bounds[v] << endl;
			// }
			for(unsigned int it = 0; it < subproblem_prime.F_ZERO.size(); ++it){
				int j = it % VEHICLES;
				int i = (it - j)/VEHICLES;
				if((subproblem_prime.F_ZERO[it] == 1) && (F_ZERO[it] == 0)){
					vars.add(X[i][j]);
					bounds.add(0);
					dirs.add(IloCplex::BranchDown);
					// cout << "x[" << i << "][" << j << "] = 0" << endl;
				}
				if((subproblem_prime.F_ONE[it] == 1) && (F_ONE[it] == 0)){
					vars.add(X[i][j]);
					bounds.add(1);
					dirs.add(IloCplex::BranchUp);
					// cout << "x[" << i << "][" << j << "] = 1" << endl;
				}
			}
			makeBranch(vars, bounds, dirs, getObjValue());
		}
	}
	
	// cout << "end branch" << endl;
}

ILOUSERCUTCALLBACK6(cutSeparation, IloRangeArray, cuts,IloNumVarMatrix &, X, IloNumVarMatrix &, Y,
					const Instance &, instance, GraphMaps, graphComp, IloNum &, currentOF) {
	
	
	clock_t tStart = clock();
	countCallback++;
	if(countCallback >= nextCallback){
		// cout << "Enter" << endl;
		countCallback = 0;
		bool foundStrCap = false;
		bool foundKTree = false;
		bool foundBiclique = false;
		for (IloInt i = 0; i < cuts.getSize(); ++i) {
			IloRange& cut = cuts[i];
			IloNum const lhs = getValue(cut.getExpr());
			if (lhs < cut.getLB() - EPS || lhs > cut.getUB() + EPS ) {
				// cout << "Adding: " << cut << " [lhs = " << lhs << "]" << endl;
				add(cut);
				cutAdded++;
				foundStrCap = true;
				nextCallback = 1;
			}
		}
		// if no strCap cuts have been added, search for k-tree cuts
		if(!foundStrCap){
			
			int CLIENTS = instance.getNbClients();
			int VEHICLES = instance.getNbVehicles();
			int STOPS = instance.getNbStops();
			int CAPACITY = instance.getCap();
			IloEnv masterEnv = getEnv();
			
			/******************************************************************/
			/*  					GET CURRENT SOLUTION 					  */
			/******************************************************************/
			IloNumMatrix X_(masterEnv, CLIENTS);
			for (int j = 0; j < CLIENTS; ++j){
				X_[j] = IloNumArray(masterEnv, VEHICLES);
				getValues(X_[j], X[j]);
			}
			IloNumMatrix Y_(masterEnv, STOPS);
			for (int i = 0; i < STOPS; ++i){
				Y_[i] = IloNumArray(masterEnv, VEHICLES);
				getValues(Y_[i], Y[i]);
			}
			
			/******************************************************************/
			/*  					FIND A VIOLATED K-TREE INEQ : 			  */
			/*							x(S) <= d.y(S)						  */
			/******************************************************************/
			// FOR EACH VEHICLE AND EACH CONNECTED INTERSECTION GRAPH, FIND A CAP+1 MIN-TREE:
			for (int k = 0; k < VEHICLES; ++k){
				for (int i = 0; i < (int)graphComp.g.size(); ++i){
					// std::cout << "Intersection: " << intersection << ". Vehicle: " << k << std::endl;
					double charge = 0.0;
					IloInt rhs = 0;
					for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
						int client = (*(graphComp.IdEdge[i]))[e];
						charge += X_[client][k];
					}
					if (charge >= 0.1){
						// CREATE WEIGHTS:
						ListGraph::EdgeMap<double> costE(*(graphComp.g[i]), 0.0);
						ListGraph::NodeMap<double> costN(*(graphComp.g[i]), 0.0);
					
						//SET WEIGHTS:
						for (ListGraph::NodeIt n(*(graphComp.g[i])); n != INVALID; ++n){
							int station = (*(graphComp.IdNode[i]))[n];
							costN[n] = -Y_[station][k];
						}
						for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
							int client = (*(graphComp.IdEdge[i]))[e];
							costE[e] = X_[client][k];
						}
						
						// K TREE SET STARTS EMPTY
						ListGraph::EdgeMap<bool> cutEdges(*(graphComp.g[i]), false);
						ListGraph::NodeMap<bool> cutNodes(*(graphComp.g[i]), false); 		//all nodes in tree since it's a spanning tree
						vector<int> innerPoints;
						int nbInnerNodes = 0;	
						
						double diffTarget = 1.0;
						double minDiff = 1.0;
						int idTarget = -1;
						IloExpr lhs(masterEnv);
						double cutValue = 0.0;
						// CHOOSE FIRST EDGE
						for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
							diffTarget = std::abs(costE[e] - 0.5);
							if (diffTarget < minDiff){
								idTarget = graphComp.g[i]->id(e);
								minDiff = diffTarget;
							}
						}
						// ADD FIRST EDGE
						cutEdges[graphComp.g[i]->edgeFromId(idTarget)] = true;
						int client = (*(graphComp.IdEdge[i]))[graphComp.g[i]->edgeFromId(idTarget)];
						lhs += X[client][k];
						cutValue += X_[client][k];
						cutNodes[graphComp.g[i]->u(graphComp.g[i]->edgeFromId(idTarget))] = true;
						cutNodes[graphComp.g[i]->v(graphComp.g[i]->edgeFromId(idTarget))] = true;
						// BUILD TREE
						for (int pos = 1; pos < CAPACITY+1; ++pos){
							double additionalCost = -2.0;
							int client = -1;
							int station = -1;
							int idClient = -1;
							// find next addition to the tree
							for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
								if(cutEdges[e] == false){
									// if is adjacent to current tree
									if( ((cutNodes[graphComp.g[i]->u(e)] == true) && (cutNodes[graphComp.g[i]->v(e)] == false)) ||  ((cutNodes[graphComp.g[i]->u(e)] == false) && (cutNodes[graphComp.g[i]->v(e)] == true)) ){
										if(cutNodes[graphComp.g[i]->u(e)] == true){
											if ( (costE[e] - costN[graphComp.g[i]->u(e)]) > additionalCost){
												additionalCost = (costE[e] - costN[graphComp.g[i]->u(e)]);
												client = (*(graphComp.IdEdge[i]))[e];
												station = (*(graphComp.IdNode[i]))[graphComp.g[i]->u(e)];
												idClient = graphComp.g[i]->id(e);
											}
										}
										else{
											if ( (costE[e] - costN[graphComp.g[i]->v(e)]) > additionalCost){
												additionalCost = (costE[e] - costN[graphComp.g[i]->v(e)]);
												client = (*(graphComp.IdEdge[i]))[e];
												station = (*(graphComp.IdNode[i]))[graphComp.g[i]->v(e)];
												idClient = graphComp.g[i]->id(e);
											}
										}
									}
									else{
										if( ((cutNodes[graphComp.g[i]->u(e)] == false) && (cutNodes[graphComp.g[i]->v(e)] == false)) ){
											if ( (costE[e] - 1) > additionalCost){
												additionalCost = (costE[e] - 1);
												client = (*(graphComp.IdEdge[i]))[e];
												idClient = graphComp.g[i]->id(e);
											}
										}
									}
								}
							}
							if (client != -1){
								// add the additional edge
								cutEdges[graphComp.g[i]->edgeFromId(idClient)] = true;
								lhs += X[client][k];
								cutValue += X_[client][k];
								if( ((cutNodes[graphComp.g[i]->u(graphComp.g[i]->edgeFromId(idClient))]) == false) && ((cutNodes[graphComp.g[i]->v(graphComp.g[i]->edgeFromId(idClient))]) == false) ){
									rhs += 1;
									// cout << "chose non-adjacent" << endl;
								}
								else{
									lhs -= Y[station][k];
									cutValue -= Y_[station][k];
									nbInnerNodes++;
									innerPoints.push_back(station);
								}
								cutNodes[graphComp.g[i]->u(graphComp.g[i]->edgeFromId(idClient))] = true;
								cutNodes[graphComp.g[i]->v(graphComp.g[i]->edgeFromId(idClient))] = true;
							}
							else{
								// cout << "Try another" << endl;
								goto endcut;
							}
						}
						//try to lift
						if (nbInnerNodes == 2){
							int innerOrig = -1;
							int innerDest = -1;
							if (innerPoints[0] < innerPoints[1]){
								innerOrig = innerPoints[0];
								innerDest = innerPoints[1];
							}
							else{
								innerOrig = innerPoints[1];
								innerDest = innerPoints[0];
							}
							
							for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
								if(cutEdges[e] == false){
									int demand = (*(graphComp.IdEdge[i]))[e];
									if ( (instance.getOrig_j(demand) == innerOrig) && (instance.getDest_j(demand) == innerDest) ){
										cutEdges[e] = true;
										lhs += X[demand][k];
										cutValue += X_[demand][k];
										// cout << "lift cut" << endl;
									}
									
								}
							}
						}
						if(cutValue >= (rhs + 0.0001)){
							try{
								//add cut
								IloRange cut = (lhs <= rhs);
								// IloCplex::CutManagement purgeable=CPX_USECUT_FILTER ;
								// cout << "Adding: " << cut << " [lhs = " << cutValue << "]" << endl;
								add(cut).end();
								foundKTree = true;
								nextCallback = 1;
								cutKTreeAdded++;
								if (rhs >= 0.9){
									cutGenKTreeAdded++;
								}
							}
							catch (...) {
								std::cout << "CAUGHT CUT ERROR. LOOK AT THE CUTTING PLANE ALGORITHM." << std::endl;
								throw;
							}
						}
						else{
							// cout << "Did not add" << endl;
						}
						endcut:
						cutValue = 0.0;
						if(foundKTree){
							i = graphComp.g.size()+1;
							k = VEHICLES+1;
						}
					}
				}
			}
		
			if(!foundKTree){
				// compare objFunc with currentOF. If objFunc <= currentOF, do nothing
				IloNum thisSolutionObjFunc = getObjValue();
				if (thisSolutionObjFunc >= currentOF + 0.000001){
					currentOF = thisSolutionObjFunc;
				// search for biclique cuts
					for (int k = 0; k < VEHICLES; ++k){
						for (int i = 0; i < (int)graphComp.g.size(); ++i){
							// std::cout << "Intersection: " << intersection << ". Vehicle: " << k << std::endl;
							
							// CREATE WEIGHTS:
							ListGraph::EdgeMap<double> costE(*(graphComp.g[i]), 0.0);
							ListGraph::NodeMap<double> costN(*(graphComp.g[i]), 0.0);
						
							//SET WEIGHTS:
							for (ListGraph::NodeIt n(*(graphComp.g[i])); n != INVALID; ++n){
								int station = (*(graphComp.IdNode[i]))[n];
								costN[n] = CAPACITY*Y_[station][k];
							}
							for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
								int client = (*(graphComp.IdEdge[i]))[e];
								costE[e] = -(CAPACITY+1)*X_[client][k];
							}
							
							// BUILD MINIMUM SPANNING TREE
							ListGraph::EdgeMap<bool> treeEdges(*(graphComp.g[i]), false);
							ListGraph::NodeMap<bool> treeNodes(*(graphComp.g[i]), true); 		//all nodes in tree since it's a spanning tree
							
							ListGraph::EdgeMap<double> Length(*(graphComp.g[i]), CLIENTS);
						
							int	sizeOfGraph=0;
							for(ListGraph::EdgeIt e(*(graphComp.g[i])); e!=INVALID; ++e){
								if (costE[e] <= -0.01){
									sizeOfGraph++;
								}
							}
							if (sizeOfGraph >= CAPACITY+2){
								
								double cutValue = 0.0;
								
								for (ListGraph::NodeIt n(*(graphComp.g[i])); n != INVALID; ++n){
									cutValue += costN[n];
								}
								kruskal(*(graphComp.g[i]), costE, treeEdges);
								
								for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
									if(treeEdges[e] == true){
										Length[e] = 1.0;
										cutValue += costE[e];
										int clientInTree = (*(graphComp.IdEdge[i]))[e];
										// cout << clientInTree << ", " ;
									}
								}
								// cout << endl;
								for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
									if(treeEdges[e] == false){
										
										// cout << "try to add edge" << (*(graphComp.IdEdge[i]))[e] << endl;
										Dijkstra<ListGraph, ListGraph::EdgeMap<double> > dijkstra(*(graphComp.g[i]), Length);
										dijkstra.run(graphComp.g[i]->u(e));
										double cycle = dijkstra.dist(graphComp.g[i]->v(e));
										// cout << "cycle = " << cycle << endl;
										if (cycle >= CAPACITY){
											// cout << "add edge" << (*(graphComp.IdEdge[i]))[e] << endl;
											Length[e] = 1.0;
											treeEdges[e] = true;
											cutValue += costE[e];
										}
									}
								}
								//evaluate cut
								if (cutValue <= -0.001){
									try{
										//add cut
										// showCut(cutClients, cutStations, k);
										
										IloNum rhs = 0.0;
										IloExpr lhs(X.getEnv());
										int nbEdges = 0;
										for (ListGraph::EdgeIt e(*(graphComp.g[i])); e != INVALID; ++e){
											if(treeEdges[e] == true){
												int client = (*(graphComp.IdEdge[i]))[e];
												nbEdges++;
												lhs += (CAPACITY+1)*X[client][k];
											}
										}
										for (ListGraph::NodeIt n(*(graphComp.g[i])); n != INVALID; ++n){
											int station = (*(graphComp.IdNode[i]))[n];
											lhs -= CAPACITY*Y[station][k];
										}
										IloRange cut = (lhs <= rhs);
										
										// IloCplex::CutManagement purgeable=CPX_USECUT_FILTER ;
										// cout << "try to add k tree cut" << endl;
										// cout << "add biclique:" << cut << "\t [lhs=" << cutValue << "]" << endl;
										add(cut).end();
										// nbcuts++;
										cutBicliqueAdded++;
										nextCallback = 1;
										lhs.end();
										
										foundBiclique = true;
										
										//try aggregated cut
										rhs = ceil((double)((nbEdges*(CAPACITY+1))/CAPACITY));
										IloExpr Aggreg(X.getEnv());
										IloNum AggValue = 0.0;
										for (int vehicle2 = 0; vehicle2 < VEHICLES; ++vehicle2){
											for (ListGraph::NodeIt n(*(graphComp.g[i])); n != INVALID; ++n){
												int station = (*(graphComp.IdNode[i]))[n];
												Aggreg += Y[station][vehicle2];
												AggValue += Y_[station][vehicle2];
											}
										}
										if (AggValue <= rhs - 0.0001){
											IloRange aggCut = (Aggreg >= rhs);
											// cout << "add aggregated:" << aggCut << endl;
											add(aggCut).end();
											cutBicliqueAdded++;
											Aggreg.end();
											nextCallback = 1;
										}
									}
									catch (...) {
										std::cout << "CAUGHT CUT ERROR. LOOK AT THE CUTTING PLANE ALGORITHM." << std::endl;
										throw;
									}
								}
							}
							if(foundBiclique){
								i = (int)graphComp.g.size()+1;
								k = VEHICLES+1;
							}
						}
					}
				}
				else{
					currentOF = 0.0;
				}
			}
		}
		if(!foundStrCap && !foundKTree && !foundBiclique){
			nextCallback = nextCallback*2;
		}
	}
	else{
		// cout << "Do not enter " << countCallback << "/" << nextCallback << endl;
	}
	cutTiming += ((double)(clock() - tStart)/CLOCKS_PER_SEC);
}
	
int main(int argc, char * argv[]){
	/******************************************************************/
	/*                    Connection with CPLEX                       */
	/******************************************************************/
	
	string instanceName = argv[1];
	cout << instanceName << endl;
	
	
	/******************************************************************/
	/*                   	Results Parameters                        */
	/******************************************************************/
	
	IloInt instanceM = 0; 						// 1-nbDemands
	IloInt instanceC = 0;						// 2-Capacity
	char instanceDelta;							// 3-Density
	IloInt instanceN = 0;						// 4-nbStations
	IloInt nbCol = 0;							// 4b-nb var
	IloInt nbRow = 0;							// 4c-nb constraints
	IloNum k_min = 0;							// 5-nb of necessary vehicles
	IloNum k_opt = 0;							// 6-optimal nb of vehicles
	IloNum TimeFinal = 0.0;						// 7- Total time
	IloNum LowerBoundFinal = 0.0;				// 8- Final Lower Bound
	IloNum UpperBoundFinal = 0.0;				// 9- Final Upper Bound
	IloNum GapFinal = 0.0;						// 10- Final Gap
	IloNum NbNodes = 0.0;						// 11- Number of Nodes on Final Tree
	IloNum NbNodesLeft = 0.0;					// 12- Number of Nodes Left to be Verified
	IloNum TimeBestSolFound = 0.0;				// 13- Time Best Solution was Found
	IloNum NbCplexCuts = 0.0;					// 14- Nb Cplex Cuts
	IloNum TimeSolveRoot = 0.0;					// 15- Time Solving Root Node
	IloNum LowerBoundRoot = 0.0;				// 16- Root Lower Bound
	IloNum UpperBoundRoot = 0.0;				// 17- Root Upper Bound
	IloNum GapRoot = 0.0;						// 18- Root Integrality Gap
	IloNum UserCuts = 0.0;						// 19- Root Integrality Gap
	IloNum TimeOnCuts = 0.0;						// 20- Root Integrality Gap
	
	
	for(int count = 1; count <= 5; count++){
		// DECLARE ENVIRONMENT.
		IloEnv env;
		try{
			// FILE results.txt HAVE THE FOLLOWING FORM :
			// NAME | OBJ FUNC ON FIRST BRANCH | GAP ON FIRST BRANCH | NB BRANCHS | OBJ FUNC | CPU TIME | TIME OF CUTS
			
			/******************************************************************/
			/*   				DECLARE MODEL AND CPLEX	 					  */
			/******************************************************************/
			IloModel model(env);
			IloCplex cplex(model);
			std::cout << "The LP and CPLEX were initialized.\n";

			/******************************************************************/
			/*	   VERIFY IF PROGRAM ARGUMENTS HAVE BEEN CORRECTLY LAUCHED	  */
			/******************************************************************/
			
			/*
			if (argc != 2){
				std::cout << std::endl;
				std::cout << "YOU MAY HAVE TO RUN PROGRAMM IN THE FOLLOWING MANNER: " << std::endl;
				std::cout << " ./main Instance.txt " << std::endl;
				throw std::exception();
			}
			*/
			/******************************************************************/
			/*                      READ INSTANCE FILE                        */
			/******************************************************************/
			
			string filePath = instanceName + "-" + std::to_string(count) + ".txt";
			Instance instance;
			instance.readInstance(filePath);
			string fileName = SplitFilename(filePath);
			
			/******************************************************************/
			/*                      SHOW INSTANCE	                          */
			/******************************************************************/
			instance.afficherInstance();
			instance.showPointsOfMaxInt();

			/******************************************************************/
			/*   				DECLARE MAIN PARAMETERS	 					  */
			/******************************************************************/
			//std::cout << "Setting parameters. " << std::endl;
			const int NB_CLIENTS = instance.getNbClients();
			const int NB_VEHICLES = instance.getNbVehicles();
			const int NB_STATIONS = instance.getNbStops();
			const int CAPACITY = instance.getCap();
			
			instanceM = NB_CLIENTS;
			instanceC = CAPACITY;
			instanceDelta = instanceName.back();
			instanceN = NB_STATIONS;
			
			IloNum timeFinish;
			IloNum duration;
			bestIncumbent = INF;
			timeBestSolutionFound = 0.0;
			timeStart = 0.0;
			lastTimeRecorded = 0.0;
			solvingRoot = 0.0;
			lbRoot = 0.0;
			ubRoot = 0.0;
			gapOnRoot = 0.0;
			cutAdded = 0.0;
			
			cutBicliqueAdded = 0.0;
			cutKTreeAdded = 0.0;
			cutGenKTreeAdded = 0.0;
			
			cutTiming = 0.0;
			countCallback = 0;
			nextCallback = 1;
			
			LOWER_BOUND.clear();
			UPPER_BOUND.clear();
			PROG.clear();
			
			int whichCase;
			if (instanceDelta == '3'){
				whichCase = 1;
			}
			else{
				if (instanceDelta == '6'){
					whichCase = 2;
				}
				else{
					if (instanceDelta == '9'){
						whichCase = 3;
					}
					else{
						whichCase = -1;
					}
				}
			}
			
			/******************************************************************/
			/*   			SETTING ASSIGNMENT VARIABLES X[a,j]				  */
			/******************************************************************/
			IloNumVarMatrix X(env, NB_CLIENTS);
			for(int a = 0; a < NB_CLIENTS; ++a){
				X[a] = IloNumVarArray(env, NB_VEHICLES);
				for(int j = 0; j < NB_VEHICLES; ++j){
					ostringstream nameAssignment;
					nameAssignment << "x(" << a << "_" << j << ")";
					
					switch (whichCase) {
						case 1: X[a][j] = IloNumVar(env, 0.0, 1.0, ILOINT, nameAssignment.str().c_str()); break;
						case 2: X[a][j] = IloNumVar(env, 0.0, 1.0, ILOINT, nameAssignment.str().c_str()); break;
						case 3: X[a][j] = IloNumVar(env, 0.0, 1.0, ILOINT, nameAssignment.str().c_str()); break;
						default: cout << "ERROR READING DELTA" << endl; 
					}
					model.add(X[a][j]);
				}
			}
			std::cout << "Assignment Variables were created.\n";

			/******************************************************************/
			/*   			SETTING STOP VARIABLES Y[i,j]					  */
			/******************************************************************/
			IloNumVarMatrix Y(env, NB_STATIONS);
			for(int i = 0; i < NB_STATIONS; ++i){
				Y[i] = IloNumVarArray(env, NB_VEHICLES);
				for(int j = 0; j < NB_VEHICLES; ++j){
					ostringstream nameStop;
					nameStop << "y(" << i << "_" << j << ")";
					switch (whichCase) {
						case 1: Y[i][j] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, nameStop.str().c_str()); break;
						case 2: Y[i][j] = IloNumVar(env, 0.0, 1.0, ILOINT, nameStop.str().c_str()); break;
						case 3: Y[i][j] = IloNumVar(env, 0.0, 1.0, ILOINT, nameStop.str().c_str()); break;
						default: cout << "ERROR READING DELTA" << endl; 
					}
					
					model.add(Y[i][j]);
				}
			}
			std::cout << "Stop Variables were created.\n";
		
			/******************************************************************/
			/*  				SETTING OBJECTIVE FUNCTION					  */
			/*					SUM{i in V, j in K}(Y[i, j])				  */
			/******************************************************************/
			IloExpr objFunction = getObjFunction(env, Y, instance);
			model.add(IloMinimize(env, objFunction));
			objFunction.end();
		
			/******************************************************************/
			/*   				SETTING ORIGINAL CONSTRAINTS 				  */
			/******************************************************************/

			/* Declare assignment constraints (1) : SUM{j in K}(x[a, j]) = 1.  */
				for(int a = 0; a < NB_CLIENTS; ++a){
					IloRange c1 = getAssignmentConst_a(env, a, X, instance);
					model.add(c1);
				}
				std::cout << "Assignment Constraints were created.\n";
			
			/* Declare capacity constraints (2) : SUM{a in Cross[i]}x[a, j] <= C. */
			for(int i = 0; i < instance.getNbPointsOfMaxInt(); ++i){
				for(int j = 0; j < NB_VEHICLES; ++j){
					int point = instance.getPointsOfMaxInt_i(i);
					IloRange c2 = getCapacityConst_i_j(env, point, j, X, instance);
					model.add(c2);
				}
			}
			std::cout << "Capacity Constraints were created.\n";

			
			// Declare stop constraints (3) : x[a, j] <= y[o[a],j].
			for(int a = 0; a < NB_CLIENTS; ++a){
				for(int j = 0; j < NB_VEHICLES; ++j){
					IloRange c3orig = getStopOrigConst_a_j(env, a, j, X, Y, instance);
					model.add(c3orig);
				}
			}
			// Declare stop constraints (3) : x[a, j] <= y[[d[a], j].
			for(int a = 0; a < NB_CLIENTS; ++a){
				for(int j = 0; j < NB_VEHICLES; ++j){
					IloRange c3dest = getStopDestConst_a_j(env, a, j, X, Y, instance);
					model.add(c3dest);
				}
			}
			std::cout << "Stop Constraints were created.\n";
			
			/******************************************************************/
			/*   			SETTING 	NEW 	CONSTRAINTS 				  */
			/******************************************************************/
			
			IloRangeArray cuts(env);
			// Declare mandatory stops constraints (5) : SUM{j in K}(y[i, j]) >= ceil(|O[i]|/C).		
			for(int i = 0; i < NB_STATIONS; ++i){
				int rhs = ceil(((double)(instance.getSizeOfStopOrig_i(i))/CAPACITY));
				if (rhs > 1){
					IloRange mandOrig = getMandOrig_i(env, i, Y, rhs, instance);
					cuts.add(mandOrig);
				}
			}
			// Declare mandatory stops constraints (5) : SUM{j in K}(y[i, j]) >= ceil(|D[i]|/C).	
			for(int i = 0; i < NB_STATIONS; ++i){
				int rhs = ceil(((double)(instance.getSizeOfStopDest_i(i))/CAPACITY));
				if (rhs > 1){
					IloRange mandDest = getMandDest_i(env, i, Y, rhs, instance);
					cuts.add(mandDest);
				}
			}
			std::cout << "Mandatory Stop Cuts were populated.\n";
			
	
			/******************************************************************/
			/*   				STRONG CAPACITY CONSTRAINTS 				  */
			/******************************************************************/
			
			// Declare strong capacity constraints (4): SUM{a in O[i]}(x[a, j]) <= C.y[i,j].
			for(int i = 0; i < NB_STATIONS; ++i){
				for(int j = 0; j < NB_VEHICLES; ++j){
					if (instance.getSizeOfStopOrig_i(i) > CAPACITY){
						IloRange cutStrongCap = getStrongCapOrig_i_j(env, i, j, X, Y, instance);
						cuts.add(cutStrongCap);
					}
				}
			}
			// Declare strong capacity constraints (4): SUM{a in D[i]}(x[a, j]) <= C.y[i,j].
			for(int i = 0; i < NB_STATIONS; ++i){
				for(int j = 0; j < NB_VEHICLES; ++j){
					if (instance.getSizeOfStopDest_i(i) > CAPACITY){
						IloRange cutStrongCap = getStrongCapDest_i_j(env, i, j, X, Y, instance);
						cuts.add(cutStrongCap);
					}
				}
			}
			std::cout << "Strong Capacity Constraints were populated." << std::endl;
		
		
			
			/******************************************************************/
			/*   				CREATE LEMON ORIGNAL GRAPH	 				  */
			/******************************************************************/
			ListGraph graph;
			IdNodeMap idNode(graph);
			IdEdgeMap idEdge(graph);
			
			// SET NODES:
			for (int s = 0; s < NB_STATIONS; ++s){
				ListGraph::Node node = graph.addNode();
				idNode[node] = s;
			}
			// SET EDGES:
			for (int a = 0; a < NB_CLIENTS; ++a){
				int o_arc =  instance.getOrig_j(a);
				int d_arc =  instance.getDest_j(a);
				ListGraph::Edge edge = graph.addEdge(graph.nodeFromId(o_arc), graph.nodeFromId(d_arc));
				idEdge[edge] = a;	
			}
			cout << "Original Graph: \n";
			showOriginalGraph(graph, idNode, idEdge);
			
			/******************************************************************/
			/*   		CREATE LEMON CONNECTED INTERSECTION GRAPHS	 		  */
			/******************************************************************/
			std::vector< std::shared_ptr<ListGraph> > vecGraph;
			std::vector< std::shared_ptr<IdNodeMap> > vecNode;
			std::vector< std::shared_ptr<IdEdgeMap> > vecEdge;
			GraphMaps graphComp;
			for (int i = 0; i < instance.getNbPointsOfMaxInt(); ++i){
				ListGraph intersectionGraph;
				IdNodeMap idIntNode(intersectionGraph);
				IdEdgeMap idIntEdge(intersectionGraph);
				
				int pointOfIntersection = instance.getPointsOfMaxInt_i(i);
				
				// SET INTERSECTION NODES:
				vector<int> intersectionStations = instance.getStationsFromClients(instance.getCross_i(pointOfIntersection));
				for (int s = 0; s < (int) intersectionStations.size(); ++s){
					ListGraph::Node node = intersectionGraph.addNode();
					idIntNode[node] = intersectionStations[s];
				}
				
				// SET INTERSECTION EDGES:
				for (int a = 0; a < (int) instance.getCross_i(pointOfIntersection).size(); ++a){
					int client = instance.getCross_i(pointOfIntersection)[a];
					int o_arc =  instance.getOrig_j(client);
					int d_arc =  instance.getDest_j(client);
					int id_o; 
					int id_d;
					for (ListGraph::NodeIt n(intersectionGraph); n != INVALID; ++n){
						if (idIntNode[n] == o_arc){
							id_o = intersectionGraph.id(n);
						}
						if (idIntNode[n] == d_arc){
							id_d = intersectionGraph.id(n);
						}
					}
					ListGraph::Edge edge = intersectionGraph.addEdge(intersectionGraph.nodeFromId(id_o), intersectionGraph.nodeFromId(id_d));
					idIntEdge[edge] = client;
				}
				cout << "Graph from intersection " << pointOfIntersection << ": \n";
				showOriginalGraph(intersectionGraph, idIntNode, idIntEdge);
				
				// SPLIT CONNECTED COMPONENTS
				IdNodeMap connectivity(intersectionGraph);
				connectedComponents(intersectionGraph, connectivity);
				int nbConnectedComp = countConnectedComponents(intersectionGraph);	
				for(int j = 0; j < nbConnectedComp; ++j){
					std::shared_ptr<ListGraph> connectedComp(new ListGraph);
					std::shared_ptr<IdNodeMap> mapN(new IdNodeMap(*connectedComp));
					std::shared_ptr<IdEdgeMap> mapE(new IdEdgeMap(*connectedComp));
					for (ListGraph::NodeIt n(intersectionGraph); n != INVALID; ++n){
						if(connectivity[n] == j){
							ListGraph::Node node = connectedComp->addNode();
							(*mapN)[node] = idIntNode[n];
						}
					}
					for (ListGraph::EdgeIt e(intersectionGraph); e != INVALID; ++e){
						if (connectivity[intersectionGraph.u(e)] == j){
							int client = idIntEdge[e];
							int o_arc =  instance.getOrig_j(client);
							int d_arc =  instance.getDest_j(client);
							int id_o; 
							int id_d;
							for (ListGraph::NodeIt n(*connectedComp); n != INVALID; ++n){
								if ((*mapN)[n] == o_arc){
									id_o = intersectionGraph.id(n);
								}
								if ((*mapN)[n] == d_arc){
									id_d = intersectionGraph.id(n);
								}
							}
							ListGraph::Edge edge = connectedComp->addEdge(connectedComp->nodeFromId(id_o), connectedComp->nodeFromId(id_d));
							(*mapE)[edge] = client;
						}
					}
					if(countEdges(*connectedComp) >= CAPACITY+1){
						// vecGraph.emplace_back(connectedComp);
						graphComp.g.emplace_back(connectedComp);
						// vecNode.emplace_back(mapN);
						graphComp.IdNode.emplace_back(mapN);
						graphComp.IdEdge.emplace_back(mapE);
						// vecEdge.emplace_back(mapE);
					}
				}
			}
			
			// Show all connected intersection graphs
			cout << "Show final graphs: \n";
			for (int i = 0; i < (int)vecGraph.size(); ++i){
				showOriginalGraph(*vecGraph[i], *vecNode[i], *vecEdge[i]);
			}
			/******************************************************************/
			/*   			EXPORT LINEAR PROGRAM TO .LP					  */
			/******************************************************************/
			string file = fileName;
			file.erase(file.end()-3, file.end());
			std::cout << file << endl;
			string lpFileName = file + "lp";
			std::cout << lpFileName << endl;
			// cplex.exportModel(lpFileName.c_str());
			/******************************************************************/
			/*   					DEFINE PARAMETERS OF MIP				  */
			/******************************************************************/
			
			
			// DISABLE CPLEX CUTS.
			// cplex.setParam(IloCplex::Cliques, 1);
			// cplex.setParam(IloCplex::Covers, 1);
			// cplex.setParam(IloCplex::DisjCuts, 1);
			// cplex.setParam(IloCplex::FlowCovers, 1);
			// cplex.setParam(IloCplex::FlowPaths, 1);
			// cplex.setParam(IloCplex::FracCuts, 1);
			// cplex.setParam(IloCplex::GUBCovers, 1);
			// cplex.setParam(IloCplex::ImplBd, 1);
			// cplex.setParam(IloCplex::MIRCuts, 1);
			// cplex.setParam(IloCplex::ZeroHalfCuts, -1);
			// cplex.setParam(IloCplex::MIPEmphasis, 2);
			
			// cplex.setParam(IloCplex::IntParam::NodeLim, 0);
			
			// ENABLE TIME LIMIT AND SINGLE THREAD
			cplex.setParam(IloCplex::TiLim, 7200);
			cplex.setParam(IloCplex::Threads, 1);
			// cplex.setParam(IloCplex::NodeLim, 0);
			// cplex.setParam(IloCplex::MIPDisplay, 3);
			//cplex.setParam(IloCplex::ClockType, 2);
			// cplex.setParam(IloCplex::ParallelMode, 1);
			// cplex.setParam(IloCplex::PreInd, 0);
			// cplex.setParam(IloCplex::Param::MIP::Strategy::Search, 1);
			// cplex.setParam(IloCplex::Param::Preprocessing::Linear, 0);
			
			// cplex.setParam(IloCplex::AggInd, 60);
		
			switch (whichCase) {
				case 1: break;
				case 2: for(int i = 0; i < NB_STATIONS; ++i){
					for(int j = 0; j < NB_VEHICLES; ++j){
						cplex.setPriority(Y[i][j], 1);
					}
				} 
				break;
				case 3: for(int i = 0; i < NB_STATIONS; ++i){
					for(int j = 0; j < NB_VEHICLES; ++j){
						cplex.setPriority(Y[i][j], 1);
					}
				} 
				break;
				default: cout << "ERROR READING DELTA" << endl; 
			}
			
			/******************************************************************/
			/*   				SOLVE LINEAR PROGRAM 						  */
			/******************************************************************/
			timeStart = cplex.getCplexTime();
			cplex.use(infoCallback(env, cplex));
			cplex.use(orbitopalFixing(env, instance, X));
			IloNum currentOF = 0.0; 
			cplex.use(cutSeparation(env, cuts, X, Y, instance, graphComp, currentOF));
			cplex.solve();
			timeFinish = cplex.getCplexTime();
			duration = timeFinish - timeStart;
			//cplex.writeSolution("sol.txt");

			/******************************************************************/
			/*   					SHOW SOLUTION							  */
			/******************************************************************/
			double fo = 0.0;
			std::cout << "Variables X : " << std::endl;
			for(int a = 0; a < NB_CLIENTS; ++a){
				for(int j = 0; j < NB_VEHICLES; ++j){
					std::cout << "X[" << a << ", " << j  << "] : " << cplex.getValue(X[a][j]) << "\t";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl << std::endl;

			std::cout << "Variables Y : " << std::endl;
			for(int i = 0; i < NB_STATIONS; ++i){
				for(int j = 0; j < NB_VEHICLES; ++j){
					std::cout << "Y[" << i << ", " << j << "] : " << cplex.getValue(Y[i][j]) << "\t";
					fo += cplex.getValue(Y[i][j]);
				}
				std::cout << std::endl;
			}
			
			
			/******************************************************************/
			/*   				TEST FOR INTEGRALITY						  */
			/******************************************************************/
			bool integralityTest = true;
			for(int a = 0; a < NB_CLIENTS; ++a){
				for(int j = 0; j < NB_VEHICLES; ++j){
					if ((cplex.getValue(X[a][j]) >= 0.001) && (cplex.getValue(X[a][j]) <= 0.999)){
						integralityTest = false;
					}
				}
			}

			for(int i = 0; i < NB_STATIONS; ++i){
				for(int j = 0; j < NB_VEHICLES; ++j){
					if ((cplex.getValue(Y[i][j]) >= 0.001) && (cplex.getValue(Y[i][j]) <= 0.999)){
						integralityTest = false;
					}
				}
			}
			
			if (integralityTest == false){
				std::ofstream errorIntegrality;
				errorIntegrality.open ("log_ErrorIntegrality.txt", std::ofstream::out | std::ofstream::app);
				errorIntegrality << filePath;
				errorIntegrality << " has fractional optimal extreme point. \n"; 
			}
			
			
			
			
			/******************************************************************/
			/*   					Recover Information						  */
			/******************************************************************/
			bool VehicleUsed;
			int usedVehicles = 0;
			for(int i = 0; i < NB_VEHICLES; ++i){
				VehicleUsed = false;
				for(int e = 0; e < NB_CLIENTS; ++e){
					if (cplex.getValue(X[e][i]) >= 0.9){
						VehicleUsed = true;
					}
				}
				if (VehicleUsed == true){
					usedVehicles++;
				}
			}
			
			IloInt cutsFromCplex = 0;
			cutsFromCplex += cplex.getNcuts(IloCplex::CutClique);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutCover);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutFlowCover);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutGubCover);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutFrac);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutMir);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutFlowPath);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutImplBd);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutDisj); 
			cutsFromCplex += cplex.getNcuts(IloCplex::CutLocalImplBd);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutZeroHalf);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutMCF);
			cutsFromCplex += cplex.getNcuts(IloCplex::CutLiftProj);
			
			
			/******************************************************************/
			/*   			Print All Information into log					  */
			/******************************************************************/
			std::ofstream ofs;
			ofs.open ("log_FINALSoutenance.txt", std::ofstream::out | std::ofstream::app);

			ofs << instanceM << "\t & \t";
			ofs << instanceC << "\t & \t";
			ofs << instanceDelta << "\t & \t";
			ofs << instanceN << "\t & \t";
			ofs << count << "\t & \t";
			ofs << cplex.getNcols() << "\t & \t";
			ofs << cplex.getNrows() << "\t & \t";
			ofs << instance.getK_min() << "\t & \t";
			ofs << usedVehicles << "\t & \t";
			
			ofs << std::setprecision(1) << std::fixed << duration << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << cplex.getBestObjValue() << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << cplex.getObjValue() << "\t & \t";
			ofs << std::setprecision(2) << std::fixed << cplex.getMIPRelativeGap()*100 << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << cplex.getNnodes()*0.001 << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << cplex.getNnodesLeft()*0.001 << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << timeBestSolutionFound << "\t & \t";
			ofs << cutsFromCplex << "\t & \t";
			ofs << cutAdded << "\t & \t";
			ofs << cutKTreeAdded << "\t & \t";
			ofs << cutGenKTreeAdded << "\t & \t";
			ofs << cutBicliqueAdded << "\t & \t";
			ofs << cutAdded + cutKTreeAdded + cutGenKTreeAdded + cutBicliqueAdded << "\t & \t";
			ofs << cutTiming << "\t & \t";
			
			ofs << std::setprecision(1) << std::fixed << solvingRoot << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << lbRoot << "\t & \t";
			ofs << std::setprecision(1) << std::fixed << ubRoot << "\t & \t";
			ofs << std::setprecision(2) << std::fixed << gapOnRoot*100 << "\t & \t";
			
			ofs << "\n";
			ofs.close();
			
			/******************************************************************/
			/*   					Get avg Information					  	  */
			/******************************************************************/
			nbCol += cplex.getNcols();
			nbRow += cplex.getNrows();
			k_min += instance.getK_min();					// 5-nb of necessary vehicles
			k_opt += usedVehicles;							// 6-optimal nb of vehicles
			TimeFinal += duration;							// 7- Total time
			LowerBoundFinal += cplex.getBestObjValue();		// 8- Final Lower Bound
			UpperBoundFinal += cplex.getObjValue();			// 9- Final Upper Bound
			GapFinal += cplex.getMIPRelativeGap()*100;		// 10- Final Gap
			NbNodes += cplex.getNnodes()*0.001;				// 11- Number of Nodes on Final Tree
			NbNodesLeft += cplex.getNnodesLeft()*0.001;		// 12- Number of Nodes Left to be Verified
			TimeBestSolFound += timeBestSolutionFound;		// 13- Time Best Solution was Found
			NbCplexCuts += cutsFromCplex;					// 14- Nb Cplex Cuts
			TimeSolveRoot += solvingRoot;					// 15- Time Solving Root Node
			LowerBoundRoot += lbRoot;						// 16- Root Lower Bound
			UpperBoundRoot += ubRoot;						// 17- Root Upper Bound
			GapRoot += gapOnRoot*100;						// 18- Root Integrality Gap
			TimeOnCuts += cutTiming;						// 19- 
			UserCuts += cutAdded;							// 20- 
			
			
			cout << "FO:" << cplex.getObjValue() << "\n";
			cout << "Time:" << timeFinish - timeStart << "\n";
			
			/******************************************************************/
			/*   				EXPORT PROGRESS TO .TXT						  */
			/******************************************************************/
			string fileProgress = fileName;
			std::cout << fileProgress << endl;
			string progressFileName = "ProgSoutenance_" + fileProgress;
			std::cout << progressFileName << endl;
			std::ofstream progress;
			progress.open (progressFileName, std::ofstream::out | std::ofstream::trunc);
			for(unsigned int it = 0; it < PROG.size(); ++it){
				progress << PROG[it] - timeStart<< "\t";
				progress << LOWER_BOUND[it] << "\t";
				progress << UPPER_BOUND[it] << "\n";
			}		
			progress.close();
		}
		catch(IloException &excep){
			cerr << endl << "CAUGHT ILOEXCEPTION: " << excep << endl;
		}
		catch(...) {
			cerr << endl << "BIG FUCKING ERROR " << endl;
		}
		
		/******************************************************************/
		/*   					CLOSE EVERYTHING						  */
		/******************************************************************/
		
		env.end();
	}
	
	
	nbCol = nbCol/5;
	nbRow = nbRow/5;
	k_min = k_min/5;							// 5-nb of necessary vehicles
	k_opt = k_opt/5;							// 6-optimal nb of vehicles
	TimeFinal = TimeFinal/5;					// 7- Total time
	LowerBoundFinal = LowerBoundFinal/5;		// 8- Final Lower Bound
	UpperBoundFinal = UpperBoundFinal/5;		// 9- Final Upper Bound
	GapFinal = GapFinal/5;						// 10- Final Gap
	NbNodes = NbNodes/5;						// 11- Number of Nodes on Final Tree
	NbNodesLeft = NbNodesLeft/5;				// 12- Number of Nodes Left to be Verified
	TimeBestSolFound = TimeBestSolFound/5;		// 13- Time Best Solution was Found
	NbCplexCuts = NbCplexCuts/5;				// 14- Nb Cplex Cuts
	TimeSolveRoot = TimeSolveRoot/5;			// 15- Time Solving Root Node
	LowerBoundRoot = LowerBoundRoot/5;			// 16- Root Lower Bound
	UpperBoundRoot = UpperBoundRoot/5;			// 17- Root Upper Bound
	GapRoot = GapRoot/5;						// 18- Root Integrality Gap
	TimeOnCuts = TimeOnCuts/5;					// 19- 
	UserCuts = UserCuts/5;						// 20- 
	
	
	std::ofstream outfile;
	outfile.open ("results_Soutenance.txt", std::ofstream::out | std::ofstream::app);

	outfile << instanceM << "\t & \t";
	outfile << instanceC << "\t & \t";
	outfile << instanceDelta << "\t & \t";
	outfile << instanceN << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << nbCol << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << nbRow << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << k_min << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << k_opt << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << TimeFinal << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << LowerBoundFinal << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << UpperBoundFinal << "\t & \t";
	outfile << std::setprecision(2) << std::fixed << GapFinal << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << NbNodes << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << NbNodesLeft << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << TimeBestSolFound << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << NbCplexCuts << "\t & \t";
	outfile << std::setprecision(2) << std::fixed << TimeSolveRoot << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << LowerBoundRoot << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << UpperBoundRoot << "\t & \t";
	outfile << std::setprecision(2) << std::fixed << GapRoot << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << UserCuts << "\t & \t";
	outfile << std::setprecision(1) << std::fixed << TimeOnCuts;
	
	outfile << "\n";
	outfile.close();
	
	return 0;
}

