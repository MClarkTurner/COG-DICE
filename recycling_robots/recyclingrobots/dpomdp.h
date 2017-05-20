/*
 Madison Clark-Turner
 dpomdp.h
 7/25/2016
 */
#ifndef DPOMDP_H
#define DPOMDP_H

#include <string>
#include <vector>
#include <random>
#include <iostream>

#include <fstream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <cfloat>

using namespace std;

#define DEBUG 0

class DPOMDP{
protected:
	size_t _timeStep = 0;
	default_random_engine _generator;

	size_t _dimension;
    
	double* _minValues;// x and y
	double* _maxValues;// x and y
    
	size_t _numAgents = 0; // The number of Taggers in the simulation
	vector<double>* _curState = NULL; // index 0 is the evader, all others are taggers
	double* _obsError = NULL; // the degree of observation error for each agent
	double* _transitionError = NULL; // the degree of error in the agent transtioning from one state to another
	string* _macroAction = NULL; // The macro action being performed by each agent
    string* _lastAction = NULL; // The last action that was performed by each agent
	size_t* _macroDuration = NULL; // the time spent performing the current macro action
	vector<string>* _actions = NULL; // the available actions for each agent
    
    vector<double> _rewards;
    
	void readValuesFromFile(string);
	virtual void restart();
	virtual void performActions();

public:
	DPOMDP(){};
	DPOMDP(size_t numAgents, size_t dimension,
		double* minValues, double* maxValues,
        double* obsError, double* transitionError, vector<string>* actions);
	DPOMDP(string);
	~DPOMDP();
    
	virtual vector<double>* Step(string* jointaction);// pass in joint action and return joint obs
	virtual double Reward(string* jointaction);// pass in joint action and return reward
    
	size_t getNumTaggers(){return _numAgents;};
	vector<double>* getCurrentState(){return _curState;};
    
	size_t getTimeStep(){return _timeStep;};
	void reset(){

		_timeStep = 0;
		//cerr << "Reset called" << endl;
		restart();
    };
    
	size_t getNumAgents(){return _numAgents;};
    string getLastAction(size_t a){return _lastAction[a];};
	size_t getNumActions(size_t a){return _actions[a].size();};
	string getAction(size_t a, size_t i){return _actions[a][i];};
    size_t getDimension(){return _dimension;};
	double getMinSize(size_t d=0){return _minValues[d];};
	double getMaxSize(size_t d=0){return _maxValues[d];};
};

#endif

/* File Format
 # Set environment of simulation
 minSize 0
 maxSize 10
 contStates false
 contObs false
 divisionSize 0.1
 evaderSpeed 2
 
 # Set tagger information
 numAgents 2
 tagRange 1	1
 obsError .8 .8
 transitionError .8 .8
 taggerSpeed 1 1
 
 #declare agent actions
 actions
 tag mvL1 mvR1 mvLHit mvRHit
 tag mvL1 mvR1 mvLHit mvRHit
 
 # Set reward information
 reward 5
 penalty -2
 wait -1
 
 */
