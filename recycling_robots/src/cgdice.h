// FOR THE DISTRIBUTION CASE OF CGDICE
#ifndef CGDICE_H
#define CGDICE_H

#include <list>
#include <string>
#include <fstream>
#include <vector>
#include <cfloat>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <random>

#include "../recyclingrobots/recyclingrobots.h"
#include "jointpolicygraphcont.h"

using namespace std;

typedef vector< vector< vector< vector< double > > > > DivCount;
typedef vector< vector< vector< DivisionData > > > DivFunc; 

class CGDICEPlanner{
private:
	DPOMDP* _problem;

	size_t _numberOfNodes;
	size_t _numberOfRestarts;
	size_t _numberOfIterations;
	size_t _numberOfSamples;
	size_t _numberOfRetainedSamples;
	size_t _numTransitions;//per axis
	double _learningRate;
	size_t _evalRuns;
	size_t _evalDepth;
	double _discount;
	string _outputFile;
	string _writeDist;
	string _setDist;
	
	struct JointPolicyGraph _bestPolicy;

	void initCEProbDistribution(
		vector< vector< vector< double > > >& _outputFunction,
	    vector< vector< vector< size_t > > >& _outputCount,

	    vector< vector< vector< vector< double > > > >& _transitionFunction,
	    vector< vector< vector< vector< size_t > > > >& _transitionCount,

	    DivFunc& _divisionFunction,
	    DivCount& _divisionCount,
	    default_random_engine& re
	    );
	void resetCounts(
	    vector< vector< vector< size_t > > >& _outputCount,
	    vector< vector< vector< vector< size_t > > > >& _transitionCount,
	    DivCount& divisionCount
	    );
	void updateCEProbDistribution(
		vector< vector< vector< double > > >& _outputFunction,
	    vector< vector< vector< size_t > > >& _outputCount,

	    vector< vector< vector< vector< double > > > >& _transitionFunction,
	    vector< vector< vector< vector< size_t > > > >& _transitionCount,

	   	DivFunc& _divisionFunction,
	    DivCount& _divisionCount,

	    unsigned restart,
    	unsigned iter,

	    list<struct JointPolicyGraph*> listOfBestPolicies
	    );
	/*void readCEProbDistribution(
		vector< vector< vector< double > > >& _outputFunction,
	    vector< vector< vector< size_t > > >& _outputCount,

	    vector< vector< vector< vector< double > > > >& _transitionFunction,
	    vector< vector< vector< vector< size_t > > > >& _transitionCount,
	    string filename
	    );*/
	void writeDistToFile(
		vector< vector< vector< double > > >& _outputFunction,
	    vector< vector< vector< size_t > > >& _outputCount,

	    vector< vector< vector< vector< double > > > >& _transitionFunction,
	    vector< vector< vector< vector< size_t > > > >& _transitionCount,

	   	DivFunc& _divisionFunction,
	    DivCount& _divisionCount,

	    unsigned restart, unsigned iteration
	    );
	void orderedInsert( struct JointPolicyGraph* pv, 
		list<struct JointPolicyGraph*>& l);
	size_t getTransition(vector<vector<double> >&,size_t, vector<double>&);

public:
	CGDICEPlanner(
		DPOMDP* problem,
		size_t numberOfNodes, 
		size_t numberOfRestarts,
		size_t numberOfIterations, 
		size_t numberOfSamples, 
		size_t numberOfRetainedSamples, 
		size_t numTransitions,
		double learningRate,
		size_t evalRuns,
		size_t evalDepth,
		double discount,
		string outputFileName = "",
		string writeDist = "",
		string setDist = ""

		);
	void plan();
	void printPolicy(){_bestPolicy.print(_problem, cout);};
	void writePolicy(){
		ofstream ofile;
		ofile.open(_outputFile);
		_bestPolicy.print(_problem, ofile);
		ofile.close();
	};
	void printPolicy(struct JointPolicyGraph jp){jp.print(_problem, cout);}
	double ApproximateEvaluate(struct JointPolicyGraph& jp);
};

#endif