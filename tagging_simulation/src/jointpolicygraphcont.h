// FOR THE DISTRIBUTION CASE OF CGDICE
#ifndef JOINTPOLICYGRAPH_H
#define JOINTPOLICYGRAPH_H

#include <random>
#include <cfloat>
#include <iostream>
#include <ostream>
#include <algorithm> 
#include "betadist.h"

#define FIX_ACTIONS 0
#define FIX_TRANSITIONS 0

using namespace std;

struct FSA{
    vector<size_t> nodes;
    vector< vector<size_t> > connections;
    vector< vector< double > > divisions;
    vector< vector< double > > divisionSort;

    FSA(){};
    FSA(random_device& rd,
   		default_random_engine& re,
   		vector< vector< double > > of,
    	vector< vector< vector< double > > > tf,
    	vector< vector< DivisionData > > df){
    	for (size_t n = 0; n < of.size(); n++)
	    {
	        vector<double> actDistribution = of[n];
	        discrete_distribution<size_t> distribution(actDistribution.begin(), actDistribution.end());
	        
	        if(FIX_ACTIONS){
		        if(n == 0)
		        	nodes.push_back(1);
		        else if(n == 1)
		        	nodes.push_back(2);
		        else if(n == 2)
		        	nodes.push_back(0);
		    }
		    else{
		    	nodes.push_back(distribution(rd));
		    }
	        connections.push_back(vector<size_t>());
	        for(size_t o = 0; o < tf[n].size(); o++)
	        {
	            vector<double> nnDistribution = tf[n][o];
	            discrete_distribution<size_t> distribution(nnDistribution.begin(), nnDistribution.end());
	            if(FIX_TRANSITIONS){
	            	if(o == 0)
			        	connections[n].push_back(0);
			        else if(o == 1)
			        	connections[n].push_back(2);
			        else if(o == 2)
			        	connections[n].push_back(1);
	            }
	            else{
	            	connections[n].push_back(distribution(rd));
	            }
	        }
	        divisions.push_back(vector<double>());

	        for(size_t d = 0; d < df[n].size(); d++)
	        {
	        	DivisionData divDistribution = df[n][d];
	        	double v = genValue(divDistribution);
				v -=.5;
				v *= 2 * 10;//replace 10 with maxSize
				divisions[n].push_back(v);
	        }
	        divisionSort.push_back(divisions[n]);
	        sort(divisionSort[n].begin(), divisionSort[n].end());
	    }
    }
};

struct JointPolicyGraph{
    /* A structure that contains the infinite horizon joint policy. 
    Used to display the joint policy where a finite-horizon joint policy
    pointer normally would. */
    vector<FSA> FSAs;
    double value;

    JointPolicyGraph()
    {
    	value = -DBL_MAX;
    }
    JointPolicyGraph(struct JointPolicyGraph* jpg)
    {
    	value = jpg->value;
    	FSAs = jpg->FSAs;
    }
  	JointPolicyGraph(random_device& rd,
   		default_random_engine& re,
  		vector< vector< vector< double > > > outputFunction,
  		vector< vector< vector< vector< double > > > > transFunction,
  		vector< vector< vector< DivisionData > > > divisionFunction
  		)
  	{
  		for(int a = 0; a < outputFunction.size(); a++){
  			struct FSA fsa(rd, re, outputFunction[a], transFunction[a], divisionFunction[a]);
  			FSAs.push_back(fsa);
  		}
  	}

    void print(TagSim* p, ostream& out)
    // Prints out the GDICEJointPolicy
    { 
		for(size_t i = 0; i < FSAs.size(); i++){
			out << "Agent " << i << endl;
			struct FSA fsa = FSAs[i];

			out << "node actions:";
			for(size_t n = 0; n < fsa.nodes.size(); n++)
			  out << "(" << n << ", " << p->getAction(i, fsa.nodes[n]) <<  "), ";
			out << endl;

			out << "divisions:" << endl;
			for(size_t n = 0; n < fsa.divisions.size(); n++){
				out << n << ": ";
				for(size_t d = 0; d < fsa.divisions[n].size(); d++){
					double v = fsa.divisions[n][d];
//					v -= 0.5;
//					v *= 20;
					out << fsa.divisions[n][d] << /*":" << v <<*/ ",";
					//out << p->getObservation(i, fsa.divisions[n][d]) << ", ";
				}
				out << endl;
			}

			out << "connections:" << endl;
			for(size_t n = 0; n < fsa.connections.size(); n++){
			  out << n << ": ";
			  for(size_t o = 0; o < fsa.connections[n].size(); o++)
			    out << "(" << o << ", " << fsa.connections[n][o] << "), ";
			  out << endl;
			}
	    }
	}
};

#endif
