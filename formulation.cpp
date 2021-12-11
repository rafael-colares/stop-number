#include "formulation.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>

/******************************************************************************/
/**				 getObjFunction returns the objective function : 			 **/
/**  				 	sum{i in V, j in K}(Y[i, j]) 		 			 	 **/
/******************************************************************************/
IloExpr getObjFunction(IloEnv &env, const IloNumVarMatrix &Y, const Instance &instance){
	IloExpr objFunction(env);
	for(int i = 0; i < instance.getNbStops(); ++i){
		for(int j = 0; j < instance.getNbVehicles(); ++j){
			objFunction += Y[i][j];
		}
	}
	return objFunction;
}


/*********************************************************************************/
/** getAssignmentConst_a returns the assignment constraint for specified arc a: **/
/**  						  sum{j in K}(x[a, j]) = 1						  	**/
/*********************************************************************************/
IloRange getAssignmentConst_a(IloEnv &env, const int a, const IloNumVarMatrix &X, const Instance &instance){
	IloExpr exp(env);
	for(int j = 0; j < instance.getNbVehicles(); ++j){
			exp += X[a][j];
	}
	ostringstream nameC1;
	nameC1 << "Assign(" << a << ")";
	IloRange c1(env, 1.0, exp, 1.0, nameC1.str().c_str());
	exp.end();
	return c1;
}

/************************************************************************************/
/** getCapacityConst_i_j returns the capacity constraint for vehicle j and stop i: **/
/**  					sum{a in Cross[i]}(x[a, j]) <= C 		 				   **/
/************************************************************************************/
IloRange getCapacityConst_i_j(IloEnv &env, const int i, const int j, const IloNumVarMatrix &X, const Instance &instance){
	IloExpr exp(env);
	for (int a = 0; a < ((int)(instance.getCross_i(i).size())); ++a){
		int arc = instance.getCross_i(i)[a];
		exp += X[arc][j];
	}
	ostringstream nameC2;
	nameC2 << "Capac(" << i << "_" << j << ")";
	IloRange c2(env, -IloInfinity, exp, instance.getCap(), nameC2.str().c_str());
	return c2;
}

/*************************************************************************************/
/** getStopOrigConst_a_j returns the origin stop constraint for arc a, vehicle j :	**/
/**  				 			x[a, j] <= y[o[a], j]			 					**/
/*************************************************************************************/
IloRange getStopOrigConst_a_j(IloEnv &env, const int a, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance){
	IloExpr exp(env);
	int o_a = instance.getOrig_j(a);
	exp += X[a][j];
	exp -= Y[o_a][j];
	
	ostringstream nameC3orig;
	nameC3orig << "StopO(" << a << "_" << j << ")";
	IloRange c3orig(env, -IloInfinity, exp, 0.0, nameC3orig.str().c_str());
	exp.end();
	return c3orig;
}

/*****************************************************************************************/
/** getStopDestConst_a_j returns the destination stop constraint for arc a, vehicle j:	**/
/**  				 			x[a, j] <= y[d[a], j]	 	 				     		**/
/*****************************************************************************************/
IloRange getStopDestConst_a_j(IloEnv &env, const int a, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance){
	IloExpr exp(env);
	int d_a = instance.getDest_j(a);
	exp += X[a][j];
	exp -= Y[d_a][j];
	
	ostringstream nameC3dest;
	nameC3dest << "StopD(" << a << "_" << j << ")";
	IloRange c3dest(env, -IloInfinity, exp, 0.0, nameC3dest.str().c_str());
	exp.end();
	return c3dest;
}


/*****************************************************************************************************/
/** getStrongCapOrig_i_j returns the origin strong capacity constraint for station i, vehicle j:	**/
/**  				 			SUM{a in O[i]}(x[a, j]) <= C.y[i,j]	 	 			     			**/
/*****************************************************************************************************/
IloRange getStrongCapOrig_i_j(const IloEnv &env, const int i, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance){
	IloExpr exp(env);
	for(unsigned int a = 0; a < instance.getStopOrig_i(i).size(); ++a){
		int arc = instance.getStopOrig_i(i)[a];
		exp += X[arc][j];
	}
	exp += -(instance.getCap()*Y[i][j]);
	ostringstream nameStrongCap;
	nameStrongCap << "StrCapO(" << i << "_" << j << ")";
	IloRange cutStrongCap(env, -IloInfinity, exp, 0.0, nameStrongCap.str().c_str());
	exp.end();
	return cutStrongCap;
}


IloNum getTightStrongCap(IloCplex cplex, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance){
	IloNum nbTight = 0.0;
	IloNum total = 0.0;
	IloNum result = -1.0;
	for(int i = 0; i < instance.getNbStops(); ++i){
		for(int j = 0; j < instance.getNbVehicles(); ++j){
			if (instance.getSizeOfStopOrig_i(i) > instance.getCap()){
				IloNum lhs = 0.0;
				total++;
				for(unsigned int e = 0; e < instance.getStopOrig_i(i).size(); ++e){
					int dem = instance.getStopOrig_i(i)[e];
					lhs += cplex.getValue(X[dem][j]);
				}
				lhs += -(instance.getCap()*(cplex.getValue(Y[i][j])));
				if ((lhs <= 0.1) && (lhs >= -0.1)){
					nbTight++;
				}
			}
			if (instance.getSizeOfStopDest_i(i) > instance.getCap()){
				IloNum lhs = 0.0;
				total++;
				for(unsigned int e = 0; e < instance.getStopDest_i(i).size(); ++e){
					int dem = instance.getStopDest_i(i)[e];
					lhs += cplex.getValue(X[dem][j]);
				}
				lhs += -(instance.getCap()*(cplex.getValue(Y[i][j])));
				if ((lhs <= 0.1) && (lhs >= -0.1)){
					nbTight++;
				}
			}
		}
	}
	if (total >= 0.9){
		result = nbTight/total;
	}
	result = result*100;
	return result;
}

/*********************************************************************************************************/
/** getStrongCapDest_i_j returns the destination strong capacity constraint for station i, vehicle j:	**/
/**  				 			SUM{a in D[i]}(x[a, j]) <= C.y[i,j]	 	 			     				**/
/*********************************************************************************************************/
IloRange getStrongCapDest_i_j(const IloEnv &env, const int i, const int j, const IloNumVarMatrix &X, const IloNumVarMatrix &Y, const Instance &instance){
	IloExpr exp(env);
	for(unsigned int a = 0; a < instance.getStopDest_i(i).size(); ++a){
		int arc = instance.getStopDest_i(i)[a];
		exp += X[arc][j];
	}
	exp += -(instance.getCap()*Y[i][j]);
	ostringstream nameStrongCap;
	nameStrongCap << "StrCapD(" << i << "_" << j << ")";
	IloRange cutStrongCap(env, -IloInfinity, exp, 0.0, nameStrongCap.str().c_str());
	exp.end();
	return cutStrongCap;
}

/******************************************************************************************************/
/** getMandOrig_i returns the origin mandatory stop constraint for specified station i :			 **/
/**  						 	sum{k in K}(Y[i, j]) >= ceil(|O[i]|/Cap)	 				 	 	 **/
/******************************************************************************************************/
IloRange getMandOrig_i(const IloEnv &env, const int i, const IloNumVarMatrix &Y, int rhs,const Instance &instance){
	IloExpr exp(env);
	for(int j = 0; j < instance.getNbVehicles(); ++j){	
		exp += Y[i][j];
	}
	ostringstream nameMandCut;
	nameMandCut << "MandOrig(" << i << ")";
	IloRange mandCut(env, rhs, exp, IloInfinity, nameMandCut.str().c_str());
	exp.end();
	return mandCut;
}

/******************************************************************************************************/
/** getMandDest_i returns the destination mandatory stop constraint for specified station i :		 **/
/**  						 	sum{k in K}(Y[i, j]) >= ceil(|D[i]|/Cap)	 				 	 	 **/
/******************************************************************************************************/
IloRange getMandDest_i(const IloEnv &env, const int i, const IloNumVarMatrix &Y, int rhs, const Instance &instance){
	IloExpr exp(env);
	for(int j = 0; j < instance.getNbVehicles(); ++j){			
		exp += Y[i][j];
	}
	ostringstream nameMandCut;
	nameMandCut << "MandDest(" << i << ")";
	IloRange mandCut(env, rhs, exp, IloInfinity, nameMandCut.str().c_str());
	exp.end();
	return mandCut;
}

/**********************************************************************************************/
/** getSymmetry1_a returns the first symmetry-breaking constraint for specified arc a :		 **/
/**  							 		sum{j in 0..a}x[a,j] = 1.			 			 	 **/
/**********************************************************************************************/
IloRange getSymmetry1_a(IloEnv &env, const int a, const IloNumVarMatrix &X, const Instance &instance){
	IloExpr exp(env);	
	for(int j = 0; j <= a; ++j){
		exp += X[a][j];
	}
	ostringstream nameSymm;
	nameSymm << "Symm1(" << a << ")";
	IloRange symmetry(env, 1.0, exp, 1.0, nameSymm.str().c_str());
	exp.end();
	return symmetry;
}

/**********************************************************************************************/
/** getSymmetry2_a returns the second symmetry-breaking constraint for arc a and vehicle j:	 **/
/**  					sum{i in j..a}x[a,i] <= sum{u in j-1..a-1}x[u,j-1].	 			 	 **/
/**********************************************************************************************/
IloRange getSymmetry2_a_j(IloEnv &env, const int a, const int j, const IloNumVarMatrix &X, const Instance &instance){
	IloExpr exp(env);	
	for(int i = j; i <= a; ++i){
		exp+=X[a][i];
	}
	for(int u = j-1; u <= a-1; ++u){
		exp -= X[u][j-1];
	}
	ostringstream nameSymm2;
	nameSymm2 << "Symm2(" << a << "_" << j << ")";
	IloRange symmetry(env, -IloInfinity, exp, 0.0, nameSymm2.str().c_str());
	exp.end();
	return symmetry;
}


