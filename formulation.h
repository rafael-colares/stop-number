#ifndef __formulation__h
#define __formulation__h

#include <ilcplex/ilocplex.h>
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

typedef IloArray<IloNumVarArray> IloNumVarMatrix;
typedef IloArray<IloNumVarMatrix> IloNumVarMatrix3D;
typedef IloArray<IloNumArray> IloNumMatrix;
typedef IloArray<IloNumMatrix> IloNumMatrix3D;
using namespace std;

//Objective Funcion
IloExpr getObjFunction(IloEnv &env, const IloNumVarMatrix &S, const Instance &instance);

// Original Constraints
IloRange getAssignmentConst_a(IloEnv &env, const int a, const IloNumVarMatrix &X, const Instance &instance);
IloRange getCapacityConst_i_j(IloEnv &env, const int i, const int j, const IloNumVarMatrix &X, const Instance &instance);
IloRange getStopOrigConst_a_j(IloEnv &env, const int a, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance);
IloRange getStopDestConst_a_j(IloEnv &env, const int a, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance);

// New Constraints
IloRange getStrongCapOrig_i_j(const IloEnv &env, const int i, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance);
IloRange getStrongCapDest_i_j(const IloEnv &env, const int i, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance);
IloRange getMandOrig_i(const IloEnv &env, const int i, const IloNumVarMatrix &Y, int rhs,const Instance &instance);
IloRange getMandDest_i(const IloEnv &env, const int i, const IloNumVarMatrix &Y, int rhs,const Instance &instance);
// Get Tightness
IloNum getTightStrongCap(IloCplex cplex, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance);

// Symmetry-Breaking Constraints
IloRange getSymmetry1_a(IloEnv &env, const int a, const IloNumVarMatrix &X, const Instance &instance);
IloRange getSymmetry2_a_j(IloEnv &env, const int a, const int j, const IloNumVarMatrix &X, const Instance &instance);
#endif
