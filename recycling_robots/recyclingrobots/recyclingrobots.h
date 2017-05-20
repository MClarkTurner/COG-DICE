#ifndef RECYCLINGROBOTS_H
#define RECYCLINGROBOTS_H

#include "dpomdp.h"

class RecyclingRobots: public DPOMDP {
private:
	void restart();
	void performActions();

	void readValuesFromFile(string filename);

public:
	RecyclingRobots(size_t numAgents,
		double* minValues, double* maxValues,
        double* obsError, double* transitionError, vector<string>* actions) 
		: DPOMDP(numAgents, 1, minValues, maxValues, obsError, transitionError, actions){};
	
	RecyclingRobots(string filename): DPOMDP()
	{
		//cerr << "RR Constructor" << endl;
		readValuesFromFile(filename);

		// initialize agent positions
		//_curState = new vector<double>[_numAgents];
	    
		restart();
	};
	

	vector<double>* Step(string* jointaction);
	double Reward(string* jointaction);
};

#endif