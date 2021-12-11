#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>      
#include <stdlib.h>    
#include <time.h>    
#include <sstream>
#include <cmath>
using namespace std;

int main () {
	int NbVehicules;
	int NbStops;
	int NbWaitingLaps = 0;
	srand(time(NULL));
	for (int cap = 2; cap <= 8; cap+=3){
		for (int NbOperations = 30; NbOperations <= 60; NbOperations+=5){
			for (int avgDegree = 3; avgDegree <= 9; avgDegree+=3){
				for (int vz = 1; vz <= 5; ++vz){
					int orig[NbOperations];
					int dest[NbOperations];
					int load[NbOperations];
					int realDest[NbOperations];
					NbStops = floor(2*NbOperations/avgDegree);
					int Int[NbStops];
					for (int j = 0; j < NbStops; ++j){
						Int[j] = 0;
					}
					for (int x = 0; x < NbOperations; ++x){
						orig[x] = (rand()%(NbStops));
						do{
							dest[x] = (rand()%(NbStops));
						}while(orig[x] == dest[x]);
						load[x] = 1;
						if (orig[x] > dest[x]){
							realDest[x] = orig[x];
							orig[x] = dest[x];
						}
						else{
							realDest[x] = dest[x];
						}
						for (int j = orig[x]; j < realDest[x]; ++j){
							Int[j]++;
						}
					}
					double maxInt = 0;
					for (int j = 0; j < NbStops; ++j){
						if (Int[j] > maxInt){
							maxInt = Int[j];
						}
					}
					NbVehicules = NbOperations;
					ofstream myfile;
					stringstream sstm;
				
					sstm << "GEN_" << cap << "_"<< NbOperations << "_" << avgDegree << "-" << vz << ".txt";
					
					string fichier = sstm.str();
					cout << "./main Instances/"<< fichier << " int a" << endl;
					myfile.open (fichier.c_str());
					myfile << "# nb_stops nb_waiting_laps nb_vehicules nb_operations capacity \n";
					myfile << NbStops << " " << NbWaitingLaps << " " << NbVehicules << " " << NbOperations << " " << cap << "\n";
					myfile << "#origin destination load \n";
					for (int x = 0; x < NbOperations; ++x){
						myfile << orig[x] << " " << realDest[x] << " " << load[x] << "\n";
					}
					myfile.close();
				}	
			}				
		}
	}
	
	return 0;
}
