#ifndef __graph__h
#define __graph__h

#include <ilcplex/ilocplex.h>
#include <lemon/list_graph.h>
#include <lemon/concepts/graph_components.h>
#include <lemon/connectivity.h>
#include <lemon/concepts/graph.h>
#include <cfloat>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <float.h>
#include <sstream>
#include <algorithm>
#include <vector> 
#include <limits>
#include "instance.h"

#define INF DBL_MAX

using namespace lemon;

typedef IloArray<IloNumVarArray> IloNumVarMatrix;
typedef IloArray<IloNumVarMatrix> IloNumVarMatrix3D;
typedef IloArray<IloNumArray> IloNumMatrix;
typedef IloArray<IloNumMatrix> IloNumMatrix3D;
using namespace std;

void showOriginalGraph(const ListGraph &g, const ListGraph::NodeMap<int> &map, const ListGraph::EdgeMap<int> &edgeMap);
void createGraph_i(const Instance &p, int i, ListGraph *g, ListGraph::NodeMap<int> *idNode);
void showGraph( ListGraph *g, ListGraph::NodeMap<int> *map);
void showGraphWithCosts( ListGraph *g, ListGraph::NodeMap<int> *map, const ListGraph::EdgeMap<double> &cost);
int getLowestSharedStop(int u, int v, const Instance &instance, const IloNumMatrix &S, const int k);
double evaluateCut(const ListGraph &g, const ListGraph::EdgeMap<bool> &tree, const IloNumMatrix &S_, const ListGraph::EdgeMap<int> &idEdge, const IloNumMatrix &Z_, const ListGraph::NodeMap<int> &idNode, const ListGraph::NodeMap<bool> &treeNodes, int k);
#endif