#ifndef __instance__h
#define __instance__h

#include <ilcplex/ilocplex.h>
#include <vector>
#include <fstream>

using namespace std;
typedef IloArray<IloNumArray>    IloNumMatrix;
typedef IloArray<IloIntArray>    IloIntMatrix;
typedef IloArray<IloIntMatrix>    IloIntMatrix3D;
typedef IloArray<IloNumVarArray> IloNumVarMatrix;
typedef IloArray<IloNumVarMatrix> IloNumVarMatrix3D;

class Instance{

private:
	int nbVehicles;
	int nbClients;
	int nbStops;
	int nbLaps;
	int capacity;
	int maxLoad;
	vector<int> Orig;
	vector<int> Dest;
	vector<int> Load;
	
	vector< vector<int> > Stop;			// vector i in I of clients
	vector< vector<int> > Cross;		// vector i in I of clients
	vector< vector<int> > StopOrig;
	vector< vector<int> > StopDest;
	
	vector<int> PointsOfMaxInt; 		// Points j in J of maximal intersection
public:

/*********************************************/
/* 				  Constructor 	    		 */
/*********************************************/

	Instance();
	
/*********************************************/
/* 					Setters  				 */
/*********************************************/

	void setNbVehicles(int pk){nbVehicles = pk;}
	void setNbStops(int pj){nbStops = pj;}
	void setNbClients(int px){nbClients = px;}
	void setNbLaps(int pl){nbLaps = pl;}
	void setCap(int pCap){capacity = pCap;}
	void setMaxLoad(int pMaxL){maxLoad = pMaxL;}
	void setOrig(int px, int pOrig){Orig[px] = pOrig;}
	void setDest(int px, int pDest){Dest[px] = pDest;}
	void setLoad(int pj, int pLoad){Load[pj] = pLoad;}
	void setSizeOfOrig(int i){Orig.resize(i);}
	void setSizeOfDest(int i){Dest.resize(i);}
	void setSizeOfLoad(int i){Load.resize(i);}
	
/*********************************************/
/* 					Getters  				 */
/*********************************************/

	int getNbVehicles() const{return nbVehicles;}						//return nb of vehicles
	int getNbStops() const{return nbStops;}								//return nb of end-points
	int getNbClients() const{return nbClients;}							//return nb of clients
	int getNbLaps() const{return nbLaps;}								//return nb of laps
	int getCap() const{return capacity;}								//return capacity
	int getMaxLoad() const{return maxLoad;}								//return max Load
	int getOrig_j(int j) const{return Orig[j];}							//return O(j)
	int getDest_j(int j) const{return Dest[j];}							//return D(j)
	int getLoad_j(int j) const{return Load[j];}							//return L(j)
	int getSizeOfStop_i(int i) const {return Stop[i].size();}			//return size of Stop(i)
	int getSizeOfCross_i(int i) const {return Cross[i].size();}			//return size of Int(i)
	int getSizeOfStopOrig_i(int i) const{return StopOrig[i].size();}	//return size of Start(i)
	int getSizeOfStopDest_i(int i) const{return StopDest[i].size();}	//return size of Finish(i)
	int getNbPointsOfMaxInt() const {return PointsOfMaxInt.size();}		//return nb of Points of Maximal Intersection 
	vector<int> getOrig() const{return Orig;}							//return vector O
	vector<int> getDest() const{return Dest;}							//return vector D
	vector< vector<int> > getStop() const{return Stop;}					//return matrix Stop
	vector< vector<int> > getCross() const{return Cross;}				//return matrix Cross
	vector< vector<int> > getStopOrig() const{return StopOrig;}			//return Start
	vector< vector<int> > getStopDest() const{return StopDest;}			//return Finish
	vector<int> getStop_i(int i) const{return Stop[i];}					//return matrix Stop(i)
	vector<int> getCross_i(int i) const{return Cross[i];}				//return matrix Cross(i)
	vector<int>  getStopOrig_i(int i) const{return StopOrig[i];}		//return Start(i)
	vector<int>  getStopDest_i(int i) const{return StopDest[i];}		//return Finish(i)
	int getPointsOfMaxInt_i(int i) const{return PointsOfMaxInt[i];}		//return PointsOfMaxInt(i)

/*********************************************/
/* 					Methodes  				 */
/*********************************************/

	void sortDemands();
	void readInstance(string);
	void afficherInstance();
	void showPointsOfMaxInt();
	void calculCross();
	void calculStop();
	void calculStopOrig();
	void calculStopDest();
	void calculPointsOfMaxInt();
	int getMaxCross();
	int getK_min();
	
	int getSizeOfCross();
	int getSizeOfStop();
	int getWeightOfStopOrig_i(int);
	int getWeightOfStopDest_i(int);
	int getNbTwins(int, int) const;
	vector<int> getStationsFromClients(const vector<int>Clients);
};

IloRange getMandCutOrig_x(IloEnv &env, const int i, const IloNumVarMatrix &S, int rhs, const Instance &instance);
IloRange getMandCutDest_x(IloEnv &env, const int i, const IloNumVarMatrix &S, int rhs, const Instance &instance);
void showSolution(const vector< vector< double> > & Z_,vector< vector< double> > & S_, int nbCars, int nbCli, int nbEnd);
IloExpr getRangCrown(IloEnv &env, const IloNumVarMatrix &S, const IloNumVarMatrix &Z, const Instance &instance);
string SplitFilename (const std::string& str);
#endif
