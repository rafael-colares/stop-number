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
	int NbOperations;
	int NbVehicules;
	int NbStops;
	int NbWaitingLaps = 0;
	int cap = ;
	srand(time(NULL));
	for (int NbOperations = 20; NbOperations <= 100; NbOperations+=10){
		for (int NbStops = 10; NbStops <= 30; NbStops+=10){
			for (int cap = 5; cap <= 15; cap+= 5){
				for (int vz = 1; vz <= 5; ++vz){
					int orig[NbOperations];
					int dest[NbOperations];
					int load[NbOperations];
					int realDest[NbOperations];
					int Int[2*NbStops];
					for (int j = 0; j < 2*NbStops; ++j){
						Int[j] = 0;
					}
					for (int x = 0; x < NbOperations; ++x){
						orig[x] = (rand()%(NbStops));
						do{
							dest[x] = (rand()%(NbStops));
						}while(orig[x] == dest[x]);
						load[x] = 1;
						if (orig[x] > dest[x]){
							realDest[x] = dest[x] + NbStops;
						}
						else{
							realDest[x] = dest[x];
						}
						for (int j = orig[x]; j < realDest[x]; ++j){
							Int[j]++;
						}
					}
					double maxInt = 0;
					for (int j = 0; j < 2*NbStops; ++j){
						if (Int[j] > maxInt){
							maxInt = Int[j];
						}
					}
					NbVehicules = ceil(maxInt/cap) + (rand()%3);
					ofstream myfile;
					stringstream sstm;
				
					sstm << "GenU_" << NbOperations << "_"<< NbStops << "_" << NbWaitingLaps << "_" << NbVehicules << "_"  << cap << "(" << vz << ")" << ".txt";
					
					string fichier = sstm.str();
					cout << fichier << endl;
					myfile.open (fichier.c_str());
					myfile << "# nb_stops nb_waiting_laps nb_vehicules nb_operations capacity \n";
					myfile << NbStops << " " << NbWaitingLaps << " " << NbVehicules << " " << NbOperations << " " << cap << "\n";
					myfile << "#origin destination load \n";
					for (int x = 0; x < NbOperations; ++x){
						myfile << orig[x] << " " << dest[x] << " " << load[x] << "\n";
					}
					myfile.close();
				}	
			}				
		}
	}
	
	return 0;
}
