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
	int cap = 2;
	srand(time(NULL));
	for (int NbOperations = 10; NbOperations <= 100; NbOperations+=10){
		for (int NbStops = 10; NbStops <= 50; NbStops+=5){
			for (int vz = 0; vz < 10; ++vz){
				int instance[NbOperations][3];
				int Int[NbStops];
				int Bip = floor((double)(NbStops)/2);
				for (int j = 0; j < NbStops; ++j){
					Int[j] = 0;
				}
				for (int x = 0; x < NbOperations; ++x){
					instance[x][0] = (rand()%(Bip));
					instance[x][1] = (rand()%(NbStops-1 - Bip)) + Bip;
					instance[x][2] = 1;
					for (int j = instance[x][0]; j < instance[x][1]; ++j){
						Int[j]++;
					}
				}
				double maxInt = 0;
				for (int j = 0; j < NbStops; ++j){
					if (Int[j] > maxInt){
						maxInt = Int[j];
					}
				}
				NbVehicules = ceil(maxInt/cap);
				ofstream myfile;
				stringstream sstm;
			
				sstm << "BipU_" << NbOperations << "_"<< NbStops << "_" << NbWaitingLaps << "_" << NbVehicules << "_"  << cap << "(" << vz << ")" << ".txt";
				
				string fichier = sstm.str();
				cout << fichier << endl;
				myfile.open (fichier.c_str());
				myfile << "# nb_stops nb_waiting_laps nb_vehicules nb_operations capacity \n";
				myfile << NbStops << " " << NbWaitingLaps << " " << NbVehicules << " " << NbOperations << " " << cap << "\n";
				myfile << "#origin destination load \n";
				for (int x = 0; x < NbOperations; ++x){
					myfile << instance[x][0] << " " << instance[x][1] << " " << instance[x][2] << "\n";
				}
				myfile.close();
			}		
		}
	}
	
	return 0;
}
