#include "instance.h"
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

#define INF DBL_MAX

/**************************************************************/
/* 					 	Constructor 						  */
/**************************************************************/
Instance::Instance(){
	setNbVehicles(0);
	setNbStops(0);
	setNbClients(0);
	setCap(0);
}

/**************************************************************/
/* 					 	Read instance  						  */
/**************************************************************/
void Instance::readInstance(string file){
	string line;
	int originClient;
	int destinationClient;
	int loadClient;
	
	ifstream myfile (file.c_str());
	if (myfile.is_open()){
		
		// ----- Read general parameters ----- //
		getline(myfile, line);
		while (line[0] == '#') getline(myfile, line);
		istringstream linestream(line);
		linestream >> nbStops >> nbLaps >> nbVehicles >> nbClients >> capacity;
		
		// ---------- Read clients ----------- //
		while(getline(myfile, line)){
			while (line[0] == '#') getline(myfile, line);
			istringstream linestream(line);
			linestream >> originClient >> destinationClient >> loadClient;
			if (destinationClient <= originClient){
				Orig.push_back(destinationClient);
				Dest.push_back(originClient);
			}
			else{
				Orig.push_back(originClient);
				Dest.push_back(destinationClient);
			}
			Load.push_back(loadClient);
		}
		
		setNbStops(nbStops);
		maxLoad = capacity;
		cout << "NbVehicles = " << nbVehicles;
		
		sortDemands();
		// ----- Calculate undirect parameters ----- //
		calculStop();
		calculCross();
		calculStopOrig();
		calculStopDest();
		calculPointsOfMaxInt();
		
		int divisor = floor( (double)(capacity)/2 ) + 1;
		nbVehicles = ceil( ((double)(nbClients))/divisor );
		cout << "Instance was read.\n";
		myfile.close();
	}
	else cout << "Unable to open file" << endl; 
}
	
/**************************************************************/
/* 					 	Show instance  						  */
/**************************************************************/
void Instance::afficherInstance(){
	std::cout << "----------------------------" << std::endl;
	std::cout << "\t Mon Instance" << endl;
	std::cout << "----------------------------" << std::endl;
	std::cout << "Stop-Points : " << getNbStops() << std::endl;
	std::cout << "Clients: " << getNbClients() << std::endl;
	std::cout << "Max Waiting Laps: " << getNbLaps() << std::endl;
	std::cout << "Vehicles : " << getNbVehicles() << std::endl;
	std::cout << "Capacity: " << getCap() << std::endl;
	std::cout << "Max Cross: " << getMaxCross() << std::endl;
	for (int j = 0; j < getNbClients(); j++){
		cout << "O[" << j << "]: " << getOrig_j(j) << "  D[" << j << "]: " << getDest_j(j) << endl;
	}
	cout << endl;
	for(unsigned int i = 0; i < Stop.size(); ++i){
		cout << "Stop(" << i <<") : {";
		for(vector<int>::const_iterator it = Stop[i].begin(); it != Stop[i].end(); it++){
			cout << *it << ", ";
		}
		cout << "}" << endl;
	}
	cout << endl;
	for(unsigned int i = 0; i < Cross.size(); ++i){
		cout << "Cross(" << i <<") : {";
		for(vector<int>::const_iterator it = Cross[i].begin(); it != Cross[i].end(); it++){
			cout << *it << ", ";
		}
		cout << "}" << endl;
	}
}

/**************************************************************/
/* 							Sort demands 					  */
/**************************************************************/
void Instance::sortDemands(){
	for (int i = 0; i < getNbClients()-1; ++i){
		int min_index = i;
		for (int j = i+1; j < getNbClients(); ++j){
			if( (getOrig_j(j) < getOrig_j(min_index)) || ( (getOrig_j(j) == getOrig_j(min_index)) && (getDest_j(j) < getDest_j(min_index)) ) ){
				min_index = j;
			}
		}
		//swap i and min_index
		int tempO = getOrig_j(i);
		int tempD = getDest_j(i);
		int tempL = getLoad_j(i);
		
		setOrig(i, getOrig_j(min_index));
		setDest(i, getDest_j(min_index));
		setLoad(i, getLoad_j(min_index));
		
		setOrig(min_index, tempO);
		setDest(min_index, tempD);
		setLoad(min_index, tempL);
	}

}

/**************************************************************/
/* 			Show Points of Maximal Intersection 			  */
/**************************************************************/
void Instance::showPointsOfMaxInt(){
	cout << "Points of maximal intersection:" << endl;
	for(vector<int>::const_iterator it = PointsOfMaxInt.begin(); it != PointsOfMaxInt.end(); it++){
		cout << *it << " ";
	}
	cout << endl;
}

/**************************************************************/
/* 				Creation of parameter Stop(j)  				  */
/**************************************************************/
void Instance::calculStop(){
	int n = getNbStops();
	int m = getNbClients();
	Stop.resize(n);
	for(int i = 0; i < n; ++i){
		for (int j = 0; j < m; j++){
			if ((getOrig_j(j) == i) || (getDest_j(j) == i)){
				Stop[i].push_back(j);
			}
		}
	}
	cout << "Parameter Stop was defined.\n";
}

/**************************************************************/
/* 				Creation of parameter Cross(i) 				  */
/**************************************************************/
void Instance::calculCross(){
	int n = getNbStops();
	int m = getNbClients();
	Cross.resize(n);
	for(int i = 0; i < n; ++i){
		for (int j = 0; j < m; j++){
			if ((getOrig_j(j) <= i) && (getDest_j(j) > i)){
				Cross[i].push_back(j);
			}
		}
	}
	cout << "Parameter Cross was defined.\n";
}

/**************************************************************/
/* 				Creation of parameter StopOrig(j)  			  */
/**************************************************************/
void Instance::calculStopOrig(){
	int n = getNbStops();
	int m = getNbClients();
	StopOrig.resize(n);
	for(int i = 0; i < n; ++i){
		for (int j = 0; j < m; j++){
			if (getOrig_j(j) == i){
				StopOrig[i].push_back(j);
			}
		}
	}
	cout << "Parameter StopOrig was defined.\n";
}

/**************************************************************/
/* 				Creation of parameter StopDest(j)  			  */
/**************************************************************/
void Instance::calculStopDest(){
	int n = getNbStops();
	int m = getNbClients();
	StopDest.resize(n);
	for(int i = 0; i < n; ++i){
		for (int j = 0; j < m; j++){
			if (getDest_j(j) == i){
				StopDest[i].push_back(j);
			}
		}
	}
	cout << "Parameter Stop was defined.\n";
}

/**************************************************************/
/* 				Creation of parameter PointsOfMaxInt		  */
/**************************************************************/
void Instance::calculPointsOfMaxInt(){
	int point = 0;
	bool hasFinished = false;
	bool hasStarted = false;
	while (point < getNbStops()){
		if (!hasStarted){
			if (getSizeOfStopOrig_i(point) != 0){
				hasStarted = true;
			}
		}
		if (!hasFinished){
			if (getSizeOfStopDest_i(point) != 0){
				hasFinished = true;
			}
		}
		if (hasFinished && hasStarted){
			PointsOfMaxInt.push_back(point-1);
			hasFinished = false;
			if (getSizeOfStopOrig_i(point) != 0){
				hasStarted = true;
			}
			else{
				hasStarted = false;
			}
		}
		point++;
	}
	/* sort PointsOfMaxInt from smallest to largest: */
	int n = (int) PointsOfMaxInt.size();
	for (int j = 0; j < n-1; j++){
		int iMin = j;
		for (int i = j+1; i < n; i++){
			int pointA = PointsOfMaxInt[i];
			int pointB = PointsOfMaxInt[iMin];
			if (Cross[pointA].size() < Cross[pointB].size()){
				iMin = i;
			}
		}
		if (iMin != j){
			int aux = PointsOfMaxInt[j];
			PointsOfMaxInt[j] = PointsOfMaxInt[iMin];
			PointsOfMaxInt[iMin] = aux;
		}
	}
}
/**************************************************************/
/*				Get Maximal Intersection		  			  */
/**************************************************************/
int Instance::getMaxCross(){
	int n = getNbStops();
	int maxCross = 0;
	
	for(int i = 0; i < n; ++i){
		int currentSizeCross = 0;
		for (unsigned int x = 0; x < Cross[i].size(); ++x){
			currentSizeCross += getLoad_j(Cross[i][x]);
		}
		if (maxCross < currentSizeCross){
			maxCross = currentSizeCross;
		}
	}
	return maxCross;
}
/**************************************************************/
/*						Get K_min				  			  */
/**************************************************************/
int Instance::getK_min(){
	double max = getMaxCross();
	double C = getCap();
	int kMin = ceil(max/C);
	return kMin;
}

/**************************************************************/
/* 					Size of Stop							  */
/**************************************************************/
int Instance::getSizeOfStop(){
	int n = getNbStops();
	int sizeOfStop = 0;
	for(int i = 0; i < n; ++i){
		sizeOfStop += Stop[i].size();
	}
	return sizeOfStop;
}

/**************************************************************/
/* 					Size of Cross				  			  */
/**************************************************************/
int Instance::getSizeOfCross(){
	int n = getNbStops();
	int sizeCross = 0;
	for(int i = 0; i < n; ++i){
		sizeCross += Cross[i].size();
	}
	return sizeCross;
}


/**************************************************************/
/* 					Weight of Start_i			  			  */
/**************************************************************/
int Instance::getWeightOfStopOrig_i(int i){
	int w = 0;
	for (unsigned int j = 0; j < StopOrig[i].size(); j++){
		w+= getLoad_j(StopOrig[i][j]);
	}
	return w;
}

/**************************************************************/
/* 					Weight of Finish_i			  			  */
/**************************************************************/
int Instance::getWeightOfStopDest_i(int i){
	int w = 0;
	for (unsigned int j = 0; j < StopDest[i].size(); j++){
		w+= getLoad_j(StopDest[i][j]);
	}
	return w;
}

int Instance::getNbTwins(int u, int v)const{
	int nbTwins = 0;
	for (int j = 0; j < nbClients; ++j){
		if ((getOrig_j(j) == u) &&  (getDest_j(j) == v)){
			nbTwins++;
		}
	}
	return nbTwins;
}
	

vector<int> Instance::getStationsFromClients(const vector<int>Clients){
	vector<int> Stations;
	for (unsigned int i = 0; i < Clients.size(); i++){
		int a = Clients[i];
		int o_a = getOrig_j(a);
		int d_a = getDest_j(a);
		if(std::find(Stations.begin(), Stations.end(), o_a) == Stations.end()) {
		/* Stations does not contain o_a */
			Stations.push_back(o_a);
		}
		if(std::find(Stations.begin(), Stations.end(), d_a) == Stations.end()) {
		/* Stations does not contain d_a */
			Stations.push_back(d_a);
		}
	}
	return Stations;
}
	
	
	
void showSolution(const vector< vector< double> > & Z_,vector< vector< double> > & S_, int nbCars, int nbCli, int nbEnd){
	cout << "Vector Z : " << endl;
	for(int k = 0; k < nbCars; ++k){
		for(int j = 0; j < nbCli; ++j){
			cout << "Z[" << j << ", " << k  << "] : " << setprecision(2) << Z_[j][k] << "\t";
		}
		cout << endl;
	}
	cout << "Vector S : " << endl;
	for(int k = 0; k < nbCars; ++k){
		for(int i = 0; i < nbEnd; ++i){
			cout << "S[" << k << ", " << i << "] : " << setprecision(2) <<  S_[k][i] << "\t";
		}
		cout << endl;
	}
}

/******************************************************************************************************/
/**			 getRangCrown returns the objective function for defining the rang of a constraint: 	 **/
/******************************************************************************************************/
IloExpr getRangCrown(IloEnv &env, const IloNumVarMatrix &S, const IloNumVarMatrix &Z, const Instance &instance){
	IloExpr objFunction(env);
	int k = 0;
	// for(int i = 0; i < instance.getNbStops(); ++i){
		// objFunction -= 2*S[k][i];
	// }
	// for(int j = 0; j < instance.getNbClients(); ++j){
		// objFunction += 3*Z[j][k];
	// }
	objFunction += Z[0][k];
			objFunction += Z[1][k];
			objFunction += Z[2][k];
			objFunction += Z[3][k];
			objFunction += Z[4][k];
			
			objFunction -= S[k][0];
			objFunction -= S[k][2];
			objFunction -= S[k][3];
	return objFunction;
}

string SplitFilename (const std::string& str)
{
  std::cout << "Splitting: " << str << '\n';
  std::size_t found = str.find_last_of("/\\");
  std::cout << " path: " << str.substr(0,found) << '\n';
  std::cout << " file: " << str.substr(found+1) << '\n';
  return str.substr(found+1);
}

