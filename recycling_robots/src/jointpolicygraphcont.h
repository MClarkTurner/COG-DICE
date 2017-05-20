// FOR THE DISTRIBUTION CASE OF CGDICE
#ifndef JOINTPOLICYGRAPH_H
#define JOINTPOLICYGRAPH_H

#include <random>
#include <cfloat>
#include <iostream>
#include <ostream>
#include <algorithm> 
#include "betadist.h"
#include "../recyclingrobots/recyclingrobots.h"

#define FIX_ACTIONS 0
#define FIX_TRANSITIONS 0

using namespace std;

struct FSA{
    vector<size_t> nodes;
    vector< vector<size_t> > connections;
    vector< vector< double > > divisions;
    vector< vector< double > > divisionSort;

    FSA()
    {}

    FSA(random_device& rd,
   		default_random_engine& re,
   		size_t dimension,
   		vector< vector< double > > of,
    	vector< vector< vector< double > > > tf,
    	vector< vector< DivisionData > > df){

    	//cerr << "t0.1" << endl;
    	for (size_t n = 0; n < of.size(); n++)
	    {
	    	//cerr << "node: " << n << endl;
	    	//cerr << "t1" << endl;
	        vector<double> actDistribution = of[n];
	        //cerr << "t1.01" << endl;
	        discrete_distribution<size_t> distribution(actDistribution.begin(), actDistribution.end());
	        //cerr << "t1.02" << endl;
	        if(FIX_ACTIONS){
		        if(n == 0)
		        	nodes.push_back(1);
		        else if(n == 1)
		        	nodes.push_back(2);
		        else if(n == 2)
		        	nodes.push_back(0);
		    }
		    else{
		    	//cerr << "t1.03" << endl;
		    	nodes.push_back(distribution(rd));
		    	//cerr << "t1.04" << endl;
		    }
		    //cerr << "t1.1" << endl;
	        connections.push_back(vector<size_t>());
	        for(size_t o = 0; o < tf[n].size(); o++)
	        {
	        	//cerr << "obs: " << o << endl;
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
	            	//cerr << "n: " << n << endl;
	            	size_t vl = distribution(rd);
	            	//cerr << "vl: " << vl << endl;
	            	connections[n].push_back(vl);
	            }
	        }
	        //cerr << "t2.1" << endl;
	        divisions.push_back(vector<double>());
	        for(size_t d = 0; d < df[n].size(); d++)
	        {
	        	//cerr << "div: " << d << endl;
	        	DivisionData divDistribution = df[n][d];
	        	double v = genValue(divDistribution);
				v *= 100;//replace 10 with maxSize
				divisions[n].push_back(v);
	        }
	        divisionSort.push_back(divisions[n]);
	        vector<double>::iterator start = divisionSort[n].begin();
	        for(size_t dim = 0; dim < dimension; dim++){
	        	//cerr << "dim: " << dim << " next: " << (divisions[n].size()/dimension) <<  endl;
	        	sort(start, start + (divisions[n].size()/dimension));
	        	start += (divisions[n].size()/dimension);
	        }
	        //cerr << "t3.1" << endl;
	    }
	    //cerr << "t8.9" << endl;
    }
};

struct JointPolicyGraph{
    /* A structure that contains the infinite horizon joint policy. 
    Used to display the joint policy where a finite-horizon joint policy
    pointer normally would. */
    vector<struct FSA> FSAs;
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
   		size_t dimension, 
  		vector< vector< vector< double > > > outputFunction,
  		vector< vector< vector< vector< double > > > > transFunction,
  		vector< vector< vector< DivisionData > > > divisionFunction
  		)
  	{
  		//cerr << "t0" << endl;
  		for(int a = 0; a < outputFunction.size(); a++){
  			struct FSA fsa(rd, re, dimension, outputFunction[a], transFunction[a], divisionFunction[a]);
  			//cerr << "t9.1" << endl;
  			FSAs.push_back(fsa);
  		}
  		//cerr << "t9.9" << endl;
  	}

    void print(DPOMDP* p, ostream& out)
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
