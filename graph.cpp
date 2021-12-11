#include "graph.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>

void createGraph_i(const Instance &p, int i, ListGraph *g, ListGraph::NodeMap<int> *idNode){
	int pointJ = i;
	vector<int> Cross = p.getCross_i(pointJ);
	for (unsigned int j = 0; j < Cross.size(); ++j){
		ListGraph::Node x = g->addNode();
		(*idNode)[x] = Cross[j];
	}
	
	for (ListGraph::NodeIt u((*g)); u != INVALID; ++u){
		for (ListGraph::NodeIt v((*g)); v != INVALID; ++v){
			if ((*idNode)[u] < (*idNode)[v]){
				int o_u = p.getOrig_j((*idNode)[u]);
				int o_v = p.getOrig_j((*idNode)[v]);
				int d_u = p.getDest_j((*idNode)[u]);
				int d_v = p.getDest_j((*idNode)[v]);
				if ( (o_u == o_v) || (d_u == d_v) ){
					ListGraph::Edge e = (*g).addEdge(u, v);
				}
			}
		}
	}
}

void showOriginalGraph(const ListGraph &g, const ListGraph::NodeMap<int> &map, const ListGraph::EdgeMap<int> &edgeMap){
	std::cout << "Nodes: " << std::endl;
	for (ListGraph::NodeIt n(g); n != INVALID; ++n){
		std::cout << map[n] << " ";
	}
	std::cout << std::endl << "Edges:" << std::endl;
	for (ListGraph::EdgeIt e(g); e != INVALID; ++e){
		std::cout << edgeMap[e] << ": " << map[g.u(e)] << "--" << map[g.v(e)] << "  ";
	}
	std::cout << std::endl;
}

void showGraph( ListGraph *g, ListGraph::NodeMap<int> *map){
	std::cout << "Nodes: " << std::endl;
	for (ListGraph::NodeIt n((*g)); n != INVALID; ++n){
		std::cout << (*map)[n] << " ";
	}
	std::cout << std::endl << "Edges:" << std::endl;
	for (ListGraph::EdgeIt e((*g)); e != INVALID; ++e){
		std::cout << (*map)[(*g).u(e)] << "--" << (*map)[(*g).v(e)] << "  ";
	}
	std::cout << std::endl;
}

void showGraphWithCosts( ListGraph *g, ListGraph::NodeMap<int> *map, const ListGraph::EdgeMap<double> &cost){
	std::cout << "Nodes: " << std::endl;
	for (ListGraph::NodeIt n((*g)); n != INVALID; ++n){
		std::cout << (*map)[n] << " ";
	}
	std::cout << std::endl << "Edges:" << std::endl;
	for (ListGraph::EdgeIt e((*g)); e != INVALID; ++e){
		std::cout << "c(" << (*map)[(*g).u(e)] << "," << (*map)[(*g).v(e)] << ")= " << cost[e] << "  ";
	}
	std::cout << std::endl;
}
int getLowestSharedStop(int u, int v, const Instance &instance, const IloNumMatrix &S, const int k){
	int o_u = instance.getOrig_j(u);
	int d_u = instance.getDest_j(u);
	int o_v = instance.getOrig_j(v);
	int d_v = instance.getDest_j(v);
	// IF THEY SHARE ONLY DEST
	if (o_u != o_v){
		return d_v;
	}
	else{
		// IF THEY SHARE ONLY ORIG
		if (d_u != d_v){
			return o_v;
		}
		// IF THEY SHARE BOTH, GET LOWEST COST
		else{
			if (S[o_v][k] <= S[d_v][k]){
				return o_v;
			}
			else{
				return d_v;
			}
		}
	}
}

double evaluateCut(const ListGraph &g, const ListGraph::EdgeMap<bool> &tree, const IloNumMatrix &S_, const ListGraph::EdgeMap<int> &idEdge, const IloNumMatrix &Z_, const ListGraph::NodeMap<int> &idNode, const ListGraph::NodeMap<bool> &treeNodes, int k){
	double cutValue = 0.0;
	
	for (ListGraph::NodeIt n(g); n != INVALID; ++n){
		if (treeNodes[n] == true){
			int id = idNode[n];
			std::cout << "+ z[" << id << "][" << k << "] ";
			// cout << " - "<< Z_[id][k] ;
			cutValue -= Z_[id][k];
		}
	}
	// cout << "evaluate tree: " << endl;
	for (ListGraph::EdgeIt e(g); e != INVALID; ++e){
		if(tree[e] == true){
			int id = idEdge[e];
			// cout << " + " << cost[e]; 
			std::cout << "- s[" << id << "][" << k << "] ";
			cutValue+= S_[id][k];
		}
	}
	cout << endl;
	// cout << endl;
	return cutValue;
}
